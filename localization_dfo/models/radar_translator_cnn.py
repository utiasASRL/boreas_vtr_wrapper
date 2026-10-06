import torch
import torch.nn as nn
import torch.nn.functional as F


class RadarTranslatorCNN(nn.Module):
    def __init__(
        self,
        output_bins=2736,
        resolution=0.04381,
        sigma_bins=3.0,
        half_window=12,
    ):
        super().__init__()

        self.output_bins = output_bins
        self.resolution = resolution
        self.sigma_bins = sigma_bins
        self.half_window = half_window

        # Learnable scalar for the complete depth-waveform skip branch.
        self.depth_skip_scale = nn.Parameter(torch.tensor(0.4))

        # ============================================================
        # 1. MAIN CNN FEATURE EXTRACTOR
        # ============================================================
        # Spatial sizes:
        # 61 -> 30 -> 15 -> 7
        self.cnn_backbone = nn.Sequential(
            nn.Conv2d(1, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
        )

        # ============================================================
        # 2. POSITIONAL ENCODING
        # ============================================================
        self.num_patches = 7 * 7
        self.embed_dim = 256

        self.pos_embed = nn.Parameter(
            torch.randn(
                1,
                self.num_patches,
                self.embed_dim,
            ) * 0.02
        )

        # ============================================================
        # 3. TRANSFORMER BOTTLENECK
        # ============================================================
        transformer_layer = nn.TransformerEncoderLayer(
            d_model=self.embed_dim,
            nhead=8,
            dim_feedforward=1024,
            dropout=0.0,
            activation="gelu",
            batch_first=True,
        )

        self.transformer_encoder = nn.TransformerEncoder(
            transformer_layer,
            num_layers=4,
        )

        # ============================================================
        # 4. SEQUENCE TRANSLATOR
        # ============================================================
        # Sequence length:
        # 49 -> 171
        self.latent_translator = nn.Sequential(
            nn.ConvTranspose1d(
                in_channels=self.embed_dim,
                out_channels=64,
                kernel_size=29,
                stride=3,
                padding=1,
            ),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
        )

        # ============================================================
        # 5. 1D WAVEFORM DECODER BODY
        # ============================================================
        # Sequence lengths:
        # 171 -> 342 -> 684 -> 1368 -> 2736
        self.decoder_body = nn.Sequential(
            nn.ConvTranspose1d(
                64,
                64,
                kernel_size=4,
                stride=2,
                padding=1,
            ),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),

            nn.ConvTranspose1d(
                64,
                32,
                kernel_size=4,
                stride=2,
                padding=1,
            ),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),

            nn.ConvTranspose1d(
                32,
                16,
                kernel_size=4,
                stride=2,
                padding=1,
            ),
            nn.BatchNorm1d(16),
            nn.ReLU(inplace=True),

            nn.ConvTranspose1d(
                16,
                8,
                kernel_size=4,
                stride=2,
                padding=1,
            ),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),
        )

        # ============================================================
        # 6. EXPERIMENT 1: LEARNED PIXEL CONFIDENCE CNN
        # ============================================================
        #
        # Input:
        #     [B, 1, 61, 61]
        #
        # Output:
        #     [B, 1, 61, 61]
        #
        # Each output value is a confidence in [0, 1].
        #
        # This branch is intentionally small so Experiment 1 changes
        # only the depth-waveform prior and does not significantly
        # alter the capacity of the main network.
        self.depth_confidence_cnn = nn.Sequential(
            # Ordinary spatial convolution.
            nn.Conv2d(
                in_channels=1,
                out_channels=16,
                kernel_size=3,
                padding=1,
            ),
            nn.ReLU(inplace=True),

            # Depthwise spatial convolution:
            # each of the 16 channels is filtered separately.
            nn.Conv2d(
                in_channels=16,
                out_channels=16,
                kernel_size=3,
                padding=1,
                groups=16,
            ),
            nn.ReLU(inplace=True),

            # Pointwise channel mixing.
            nn.Conv2d(
                in_channels=16,
                out_channels=16,
                kernel_size=1,
            ),
            nn.ReLU(inplace=True),

            # Produce one confidence logit per depth pixel.
            nn.Conv2d(
                in_channels=16,
                out_channels=1,
                kernel_size=1,
            ),

            nn.Sigmoid(),
        )

        # Initialize the final confidence layer so that the initial
        # confidence is approximately:
        #
        # sigmoid(4) ~= 0.982
        #
        # Therefore the model initially behaves similarly to an
        # unweighted depth prior.
        final_confidence_conv = self.depth_confidence_cnn[-2]

        nn.init.zeros_(final_confidence_conv.weight)
        nn.init.constant_(final_confidence_conv.bias, 4.0)

        # ============================================================
        # 7. FIXED GAUSSIAN WAVEFORM KERNEL
        # ============================================================
        #
        # Registering the kernel as a buffer means:
        # - it is moved automatically with model.to(device);
        # - it is stored in checkpoints;
        # - it is not treated as a learnable parameter;
        # - it does not need to be rebuilt every forward pass.
        offsets = torch.arange(
            -half_window,
            half_window + 1,
            dtype=torch.float32,
        )

        gaussian_kernel = torch.exp(
            -0.5 * (offsets / sigma_bins) ** 2
        )

        # Peak normalization:
        # the central kernel coefficient is exactly 1.
        gaussian_kernel = (
            gaussian_kernel / gaussian_kernel.max()
        )

        gaussian_kernel = gaussian_kernel.view(1, 1, -1)

        self.register_buffer(
            "depth_gaussian_kernel",
            gaussian_kernel,
        )

        # ============================================================
        # 8. FINAL FUSION DECODER
        # ============================================================
        #
        # Decoder feature:
        #     [B, 8, 2736]
        #
        # Learned depth prior:
        #     [B, 1, 2736]
        #
        # Concatenated:
        #     [B, 9, 2736]
        self.final_fusion = nn.Sequential(
            nn.Conv1d(
                9,
                8,
                kernel_size=5,
                padding=2,
            ),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),

            nn.Conv1d(
                8,
                1,
                kernel_size=5,
                padding=2,
            ),
        )

    def batch_depth_to_waveforms_soft_local(
        self,
        patches,
        confidence,
    ):
        """
        Construct the Experiment 1 learned depth-waveform prior.

        The depth values determine the hard range-bin indices.
        The learned confidence values determine how much evidence
        each pixel contributes.

        Pipeline:
            depth
            -> hard rounded range-bin index

            confidence
            -> weighted scatter_add into range bins

            accumulated waveform
            -> fixed Gaussian convolution
            -> smooth saturation: 1 - exp(-x)

        The operation is differentiable with respect to confidence,
        but intentionally not differentiable with respect to depth
        bin assignment.

        Args:
            patches:
                Raw depth patches with shape [B, 1, H, W].

            confidence:
                Learned confidence map with shape [B, 1, H, W].

        Returns:
            Waveform tensor with shape [B, self.output_bins].
        """
        if patches.ndim != 4:
            raise ValueError(
                "Expected patches with shape [B, C, H, W], "
                f"but received {tuple(patches.shape)}."
            )

        if confidence.shape != patches[:, :1].shape:
            raise ValueError(
                "confidence must have shape [B, 1, H, W]. "
                f"Received patches shape {tuple(patches.shape)} "
                f"and confidence shape {tuple(confidence.shape)}."
            )

        batch_size = patches.shape[0]
        dtype = patches.dtype
        device = patches.device

        # Flatten the depth patch and confidence map:
        #
        # depths:      [B, H*W]
        # confidence:  [B, H*W]
        depths = patches[:, 0].reshape(batch_size, -1)
        confidence = confidence[:, 0].reshape(batch_size, -1)

        # Convert metric depth to the nearest discrete radar bin.
        #
        # The integer index is not differentiable with respect to depth,
        # which is acceptable because the input geometry is fixed.
        indices = torch.round(
            depths / self.resolution
        ).long()

        # A valid depth must:
        # - be finite;
        # - be positive;
        # - map inside the output waveform.
        valid = (
            torch.isfinite(depths)
            & (depths > 0)
            & (indices >= 0)
            & (indices < self.output_bins)
        )

        # scatter_add_ requires every supplied index to be in bounds,
        # even when the corresponding source value is zero.
        safe_indices = indices.clamp(
            min=0,
            max=self.output_bins - 1,
        )

        # Invalid pixels contribute exactly zero confidence.
        weighted_confidence = (
            confidence * valid.to(dtype)
        )

        # Initialize the hard-bin confidence histogram.
        waveform = torch.zeros(
            batch_size,
            self.output_bins,
            dtype=dtype,
            device=device,
        )

        # Sum all pixel confidences that land in the same range bin.
        #
        # Unlike binary assignment:
        #
        #     waveform[index] = 1
        #
        # this preserves the contribution and gradient from every
        # confidence value.
        waveform.scatter_add_(
            dim=1,
            index=safe_indices,
            src=weighted_confidence,
        )

        # Match the kernel dtype to the input. This is useful when
        # training with automatic mixed precision.
        kernel = self.depth_gaussian_kernel.to(
            dtype=waveform.dtype
        )

        # Smooth each batch waveform using the same fixed kernel.
        waveform = F.conv1d(
            waveform.unsqueeze(1),
            kernel,
            padding=self.half_window,
        ).squeeze(1)

        # Smoothly bound the waveform to [0, 1).
        #
        # This preserves accumulated evidence:
        #
        # x = 0 -> 0
        # x = 1 -> 0.632
        # x = 2 -> 0.865
        #
        # while preventing arbitrarily large values when many depth
        # pixels contribute to nearby range bins.
        waveform = 1.0 - torch.exp(-waveform)

        return waveform

    def forward(self, x):
        raw_patch = x

        # ============================================================
        # MAIN LEARNED BRANCH
        # ============================================================
        x = self.cnn_backbone(x)        # [B, 256, 7, 7]

        batch_size, channels, height, width = x.shape

        x = x.view(
            batch_size,
            channels,
            height * width,
        )                               # [B, 256, 49]

        x = x.permute(0, 2, 1)          # [B, 49, 256]

        x = x + self.pos_embed          # [B, 49, 256]

        x = self.transformer_encoder(x) # [B, 49, 256]

        x = x.permute(0, 2, 1)          # [B, 256, 49]

        x = self.latent_translator(x)   # [B, 64, 171]

        x = self.decoder_body(x)        # [B, 8, 2736]

        # ============================================================
        # EXPERIMENT 1 DEPTH-PRIOR BRANCH
        # ============================================================

        # Predict one confidence per input depth pixel.
        confidence = self.depth_confidence_cnn(
            raw_patch
        )                               # [B, 1, 61, 61]

        # Ensure invalid depth pixels have exactly zero confidence.
        #
        # This masking also means that the displayed confidence map
        # represents only valid geometric input.
        valid_depth = (
            torch.isfinite(raw_patch)
            & (raw_patch > 0)
        )

        confidence = (
            confidence * valid_depth.to(confidence.dtype)
        )

        # Convert the learned confidence map into a smooth waveform.
        depth_prior = (
            self.batch_depth_to_waveforms_soft_local(
                patches=raw_patch,
                confidence=confidence,
            )
        )                               # [B, 2736]

        depth_prior = depth_prior.unsqueeze(1)
                                            # [B, 1, 2736]

        # Retain the original global learnable skip scale.
        depth_prior = (
            self.depth_skip_scale * depth_prior
        )

        # ============================================================
        # LATE FUSION
        # ============================================================
        x = torch.cat(
            [x, depth_prior],
            dim=1,
        )                               # [B, 9, 2736]

        x = self.final_fusion(x)        # [B, 1, 2736]

        return x.squeeze(1)             # [B, 2736]