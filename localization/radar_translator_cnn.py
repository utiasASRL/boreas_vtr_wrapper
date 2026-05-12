import torch
import torch.nn as nn
import torch.nn.functional as F

class RadarTranslatorCNN(nn.Module):
    def __init__(self, output_bins=6848):
        super().__init__()

        # -------- 2D encoder: [B,1,21,21] -> latent --------
        self.encoder = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=3, padding=1),   # 21x21
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                              # 21 -> 10

            nn.Conv2d(32, 64, kernel_size=3, padding=1), # 10x10
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                              # 10 -> 5

            nn.Conv2d(64, 128, kernel_size=3, padding=1),# 5x5
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),

            nn.Conv2d(128, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )

        flattened_dim = 128 * 5 * 5

        # 6848 = 107 * 64, so seed length 107 then upsample x2 six times
        self.fc = nn.Linear(flattened_dim, 128 * 107)

        # -------- 1D decoder --------
        self.decoder = nn.Sequential(
            # 107 -> 214
            nn.ConvTranspose1d(128, 128, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),

            # 214 -> 428
            nn.ConvTranspose1d(128, 64, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),

            # 428 -> 856
            nn.ConvTranspose1d(64, 64, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),

            # 856 -> 1712
            nn.ConvTranspose1d(64, 32, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),

            # 1712 -> 3424
            nn.ConvTranspose1d(32, 16, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(16),
            nn.ReLU(inplace=True),

            # 3424 -> 6848
            nn.ConvTranspose1d(16, 8, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),

            # refine locally
            nn.Conv1d(8, 8, kernel_size=5, padding=2),
            nn.BatchNorm1d(8),
            nn.ReLU(inplace=True),

            nn.Conv1d(8, 1, kernel_size=5, padding=2)
        )

    def forward(self, x):
        x = self.encoder(x)                # [B,256,5,5]
        x = torch.flatten(x, start_dim=1)  # [B, 3200]
        x = self.fc(x)                     # [B, 13696]
        x = x.view(x.size(0), 128, 107)    # [B, 128, 107]
        
        x = self.decoder(x)                # [B, 1, 6848]
        x = x.squeeze(1)                   # [B, 6848]
        return x