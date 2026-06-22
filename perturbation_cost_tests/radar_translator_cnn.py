import torch
import torch.nn as nn
import torch.nn.functional as F

class RadarTranslatorCNN(nn.Module):
    def __init__(self, output_bins=2048):
        super().__init__()
        self.output_bins = output_bins

        # -------- 1. CNN Feature Extractor (Patch Generator) --------
        # Gracefully steps 61x61 down to 7x7 without zero-padding hacks
        self.cnn_backbone = nn.Sequential(
            # 61 -> 30
            nn.Conv2d(1, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            # 30 -> 15
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            # 15 -> 7
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2)
        )

        # -------- 2. Positional Encoding --------
        # We have a 7x7 grid = 49 spatial tokens. 
        # The embedding dimension is 128 (matching the CNN output channels).
        self.num_patches = 7 * 7
        self.embed_dim = 128
        
        # Learnable positional embedding tensor
        self.pos_embed = nn.Parameter(torch.randn(1, self.num_patches, self.embed_dim) * 0.02)

        # -------- 3. Transformer Bottleneck --------
        # We use a full TransformerEncoderLayer instead of raw MultiheadAttention.
        # This includes the internal Feed-Forward MLPs and LayerNorms standard in ViTs.
        transformer_layer = nn.TransformerEncoderLayer(
            d_model=self.embed_dim,
            nhead=4,
            dim_feedforward=512,
            dropout=0.3,
            activation='gelu',
            batch_first=True
        )
        # Stack 2 layers to give the attention mechanism depth
        self.transformer_encoder = nn.TransformerEncoder(transformer_layer, num_layers=2)

        # -------- 4. Sequence Translator --------
        # Maps the 49 Transformer tokens into the 171 tokens needed for the decoder
        # Formula: Out = (In - 1)*stride - 2*padding + kernel
        # 171 = (49 - 1)*3 - 2*1 + 29
        self.latent_translator = nn.Sequential(
            nn.ConvTranspose1d(
                in_channels=self.embed_dim, 
                out_channels=64, 
                kernel_size=29, 
                stride=3, 
                padding=1
            ),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True)
        )

        # -------- 5. 1D Waveform Decoder --------
        self.decoder = nn.Sequential(
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

            nn.Conv1d(8, 8, kernel_size=5, padding=2),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),

            nn.Conv1d(8, 1, kernel_size=5, padding=2),
        )

    def forward(self, x):
        # 1. Extract Local Features
        x = self.cnn_backbone(x)            # [B, 128, 7, 7]
        
        # 2. Reshape to Sequence
        B, C, H, W = x.size()
        x = x.view(B, C, H * W)             # [B, 128, 49]
        x = x.permute(0, 2, 1)              # [B, 49, 128] (Tokens, Embed_Dim)
        
        # 3. Add Spatial Awareness
        x = x + self.pos_embed              # Broadcasts [1, 49, 128] to [B, 49, 128]
        
        # 4. Global Self-Attention
        x = self.transformer_encoder(x)     # [B, 49, 128]
        
        # 5. Prepare for 1D Translation
        x = x.permute(0, 2, 1)              # [B, 128, 49] (Channels, Tokens)
        x = self.latent_translator(x)       # [B, 64, 171]
        
        # 6. Decode to Output Bins
        x = self.decoder(x)                 # [B, 1, 2736]
        x = x.squeeze(1)                    # [B, 2736]
        
        return x