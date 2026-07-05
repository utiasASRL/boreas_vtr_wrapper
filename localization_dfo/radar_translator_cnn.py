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

        # Learnable scalar for the depth waveform skip
        self.depth_skip_scale = nn.Parameter(torch.tensor(1.0))

        # -------- 1. CNN Feature Extractor (Patch Generator) --------
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

        # -------- 2. Positional Encoding --------
        self.num_patches = 7 * 7
        self.embed_dim = 256

        self.pos_embed = nn.Parameter(
            torch.randn(1, self.num_patches, self.embed_dim) * 0.02
        )

        # -------- 3. Transformer Bottleneck --------
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

        # -------- 4. Sequence Translator --------
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

        # -------- 5. 1D Waveform Decoder Body --------
        # Same as before up to [B, 8, 2736]
        self.decoder_body = nn.Sequential(
            nn.ConvTranspose1d(64, 64, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),

            nn.ConvTranspose1d(64, 32, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),

            nn.ConvTranspose1d(32, 16, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(16),
            nn.ReLU(inplace=True),

            nn.ConvTranspose1d(16, 8, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),
        )

        # -------- 6. Final Fusion Decoder --------
        # Original final block was:
        #   Conv1d(8, 8)
        #   Conv1d(8, 1)
        #
        # Now we concatenate depth waveform prior:
        #   decoder feature: [B, 8, L]
        #   depth prior:     [B, 1, L]
        #   concat:          [B, 9, L]
        self.final_fusion = nn.Sequential(
            nn.Conv1d(9, 8, kernel_size=5, padding=2),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),

            nn.Conv1d(8, 1, kernel_size=5, padding=2),
        )

    def batch_depth_to_waveforms_soft_local(self, patches, eps=1e-8):
        """
        Differentiable soft depth-to-waveform prior.

        patches: [B, 1, H, W]
        returns: [B, output_bins]

        Important:
        - No hard round-to-bin assignment.
        - Uses continuous bin coordinate u = depth / resolution.
        - Uses floor only to choose local support.
        - Uses peak-normalized Gaussian weights.
        - Uses smooth saturation to keep output in [0, 1).
        """
        B = patches.shape[0]
        device = patches.device
        dtype = patches.dtype

        depths = patches[:, 0].reshape(B, -1)  # [B, N]
        valid = depths > 0

        # Continuous range-bin coordinate
        u = depths / self.resolution  # [B, N]

        # Only used to choose local Gaussian support
        base = torch.floor(u).long()  # [B, N]

        offsets = torch.arange(
            -self.half_window,
            self.half_window + 1,
            device=device,
            dtype=torch.long,
        )  # [K]

        idx = base.unsqueeze(-1) + offsets.view(1, 1, -1)  # [B, N, K]

        in_bounds = (
            (idx >= 0)
            & (idx < self.output_bins)
            & valid.unsqueeze(-1)
        )

        # Differentiable with respect to u/depths
        diff = idx.to(dtype) - u.unsqueeze(-1)  # [B, N, K]

        # Peak-normalized Gaussian, not area-normalized
        weights = torch.exp(-0.5 * (diff / self.sigma_bins) ** 2)
        weights = weights * in_bounds.to(dtype)

        idx_safe = idx.clamp(0, self.output_bins - 1)

        waveform = torch.zeros(
            B,
            self.output_bins,
            device=device,
            dtype=dtype,
        )

        waveform.scatter_add_(
            dim=1,
            index=idx_safe.reshape(B, -1),
            src=weights.reshape(B, -1),
        )

        # Smooth bounded accumulation.
        # One perfectly centered point gives 1 - exp(-1) ~= 0.632.
        # Multiple overlapping points approach 1.
        waveform = 1.0 - torch.exp(-waveform)

        return waveform

    def forward(self, x):
        raw_patch = x

        # -------- Main learned branch --------
        x = self.cnn_backbone(x)            # [B, 256, 7, 7]

        B, C, H, W = x.size()
        x = x.view(B, C, H * W)             # [B, 256, 49]
        x = x.permute(0, 2, 1)              # [B, 49, 256]

        x = x + self.pos_embed              # [B, 49, 256]
        x = self.transformer_encoder(x)     # [B, 49, 256]

        x = x.permute(0, 2, 1)              # [B, 256, 49]
        x = self.latent_translator(x)       # [B, 64, 171]

        x = self.decoder_body(x)            # [B, 8, 2736]

        # -------- Depth-to-waveform skip branch --------
        depth_prior = self.batch_depth_to_waveforms_soft_local(raw_patch)
        depth_prior = depth_prior.unsqueeze(1)  # [B, 1, 2736]

        depth_prior = self.depth_skip_scale * depth_prior

        # -------- Late fusion --------
        x = torch.cat([x, depth_prior], dim=1)  # [B, 9, 2736]
        x = self.final_fusion(x)               # [B, 1, 2736]

        return x.squeeze(1)