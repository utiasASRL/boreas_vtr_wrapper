"""OptixDepthBackend for the multi-channel tracer copy (native/optix_range_tracer_multi, concept test).

Same interface as localization_dfo.optix_backend.OptixDepthBackend, but trace() returns [B, C, H, W]:
  bands:  closest hit inside each of C equal range bands of [0, max_range]
  layers: first hit, then the closest hit at least layer_skip m past the previous one (up to max_range)
Misses are 0. Build with: bash native/optix_range_tracer_multi/build.sh
"""
import os
import sys
from pathlib import Path

import numpy as np
import torch

from localization_dfo.optix_backend import OptixDepthBackend

BUILD_DIR = Path(os.getenv("OPTIX_MULTI_BUILD_DIR",
                           Path(__file__).resolve().parents[1] / "build" / "optix_range_tracer_multi"))


class MultiChannelDepthBackend(OptixDepthBackend):
    def __init__(self, patch_config, device, mode, channels, max_range=None, layer_skip=0.0):
        if mode not in ("bands", "layers"):
            raise ValueError(f"mode must be 'bands' or 'layers', got {mode!r}.")
        if channels < 1:
            raise ValueError("channels must be positive.")
        if mode == "bands" and not max_range:
            raise ValueError("bands mode needs a max range.")
        sys.path.insert(0, str(BUILD_DIR / "python"))
        try:
            import optix_range_tracer_multi
        except ImportError as error:
            raise RuntimeError(
                f"Could not import optix_range_tracer_multi from {BUILD_DIR / 'python'}. "
                "Build it with: bash native/optix_range_tracer_multi/build.sh"
            ) from error

        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise RuntimeError("The OptiX depth backend requires a CUDA device.")
        if self.device.index is None:
            self.device = torch.device("cuda", torch.cuda.current_device())
        self.height = int(patch_config["height"])
        self.width = int(patch_config["width"])
        self.channels = int(channels)
        self.pose_count = 0
        self.mesh_ready = False
        self.scan_ready = False
        with torch.cuda.device(self.device):
            self.tracer = optix_range_tracer_multi.OptixRangeTracer(
                width=self.width,
                height=self.height,
                theta_min=float(np.rad2deg(patch_config["theta_min"])),
                theta_max=float(np.rad2deg(patch_config["theta_max"])),
                phi_min=float(np.rad2deg(patch_config["phi_min"])),
                phi_max=float(np.rad2deg(patch_config["phi_max"])),
                min_range=float(patch_config["min_range"]),
            )
            self.tracer.set_channels(mode, self.channels, float(max_range or 1.0e16), float(layer_skip))

    def trace(self, current_transform):
        if not self.mesh_ready:
            raise RuntimeError("OptiX tracer used before mesh setup.")
        if not self.scan_ready:
            raise RuntimeError("OptiX tracer used before scan setup.")
        transform_gpu = torch.as_tensor(current_transform, device=self.device, dtype=torch.float32).contiguous()
        if transform_gpu.shape != (4, 4):
            raise ValueError(f"T_current must have shape [4, 4], got {tuple(transform_gpu.shape)}.")
        depth = self.tracer.trace(transform_gpu)
        expected_shape = (self.pose_count, self.channels, self.height, self.width)
        if tuple(depth.shape) != expected_shape:
            raise RuntimeError(f"OptiX returned shape {tuple(depth.shape)}, expected {expected_shape}.")
        return depth
