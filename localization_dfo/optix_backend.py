import numpy as np
import torch


class OptixDepthBackend:
    def __init__(self, patch_config, device):
        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise RuntimeError("The OptiX depth backend requires a CUDA device.")
        if self.device.index is None:
            self.device = torch.device("cuda", torch.cuda.current_device())
        if (int(patch_config["height"]), int(patch_config["width"])) != (61, 61):
            raise ValueError(
                "The radar translator CNN requires 61x61 patches, got "
                f"{patch_config['height']}x{patch_config['width']}."
            )
        try:
            import optix_range_tracer
        except ImportError as error:
            raise RuntimeError(
                "Could not import optix_range_tracer. Enter through scripts/run_docker.sh "
                "and build the Stage 3 extension first."
            ) from error

        self.height = int(patch_config["height"])
        self.width = int(patch_config["width"])
        self.pose_count = 0
        self.mesh_ready = False
        self.scan_ready = False
        with torch.cuda.device(self.device):
            self.tracer = optix_range_tracer.OptixRangeTracer(
                width=self.width,
                height=self.height,
                theta_min=float(np.rad2deg(patch_config["theta_min"])),
                theta_max=float(np.rad2deg(patch_config["theta_max"])),
                phi_min=float(np.rad2deg(patch_config["phi_min"])),
                phi_max=float(np.rad2deg(patch_config["phi_max"])),
            )

    def set_mesh(self, vertices, triangles):
        self._check_tensor(vertices, "vertices", torch.float32, 2)
        self._check_tensor(triangles, "triangles", torch.int32, 2)
        if vertices.shape[1] != 3 or triangles.shape[1] != 3:
            raise ValueError("OptiX vertices and triangles must have shape [N, 3] and [F, 3].")
        self.tracer.set_mesh(vertices, triangles)
        self.mesh_ready = True

    def set_scan(self, odom_transforms, radar_azimuths):
        odom_gpu = torch.as_tensor(
            odom_transforms, device=self.device, dtype=torch.float32
        ).contiguous()
        azimuths_gpu = torch.as_tensor(
            radar_azimuths, device=self.device, dtype=torch.float32
        ).contiguous()
        if odom_gpu.ndim != 3 or tuple(odom_gpu.shape[1:]) != (4, 4):
            raise ValueError(f"odom_transforms must have shape [B, 4, 4], got {tuple(odom_gpu.shape)}.")
        if azimuths_gpu.ndim != 1:
            raise ValueError(f"radar_azimuths must have shape [B], got {tuple(azimuths_gpu.shape)}.")
        if len(odom_gpu) != len(azimuths_gpu):
            raise ValueError(
                f"Odometry and radar pose counts differ: {len(odom_gpu)} != {len(azimuths_gpu)}."
            )
        self.tracer.set_scan(odom_gpu, azimuths_gpu)
        self.pose_count = len(odom_gpu)
        self.scan_ready = True

    def trace(self, current_transform):
        if not self.mesh_ready:
            raise RuntimeError("OptiX tracer used before mesh setup.")
        if not self.scan_ready:
            raise RuntimeError("OptiX tracer used before scan setup.")
        transform_gpu = torch.as_tensor(
            current_transform, device=self.device, dtype=torch.float32
        ).contiguous()
        if transform_gpu.shape != (4, 4):
            raise ValueError(f"T_current must have shape [4, 4], got {tuple(transform_gpu.shape)}.")
        depth = self.tracer.trace(transform_gpu)
        expected_shape = (self.pose_count, self.height, self.width)
        if tuple(depth.shape) != expected_shape:
            raise RuntimeError(f"OptiX returned shape {tuple(depth.shape)}, expected {expected_shape}.")
        if depth.dtype != torch.float32 or depth.device != self.device or not depth.is_contiguous():
            raise RuntimeError(
                "OptiX output must be contiguous CUDA float32 on the backend device."
            )
        return depth

    def _check_tensor(self, tensor, name, dtype, dimensions):
        if not torch.is_tensor(tensor):
            raise TypeError(f"{name} must be a torch.Tensor.")
        if tensor.device != self.device:
            raise ValueError(f"{name} must be on {self.device}, got {tensor.device}.")
        if tensor.dtype != dtype:
            raise ValueError(f"{name} must have dtype {dtype}, got {tensor.dtype}.")
        if tensor.ndim != dimensions:
            raise ValueError(f"{name} must have {dimensions} dimensions, got {tensor.ndim}.")
        if not tensor.is_contiguous():
            raise ValueError(f"{name} must be contiguous.")


def compute_optix_cost(backend, current_transform, observed_radar, model):
    if observed_radar.ndim != 2 or observed_radar.shape[0] != backend.pose_count:
        raise ValueError(
            f"Observed radar must have shape [B, bins] with B={backend.pose_count}, "
            f"got {tuple(observed_radar.shape)}."
        )
    if (
        observed_radar.device != backend.device
        or observed_radar.dtype != torch.float32
        or not observed_radar.is_contiguous()
    ):
        raise ValueError("Observed radar must be contiguous CUDA float32 on the backend device.")

    with torch.cuda.device(backend.device):
        trace_start = torch.cuda.Event(enable_timing=True)
        trace_end = torch.cuda.Event(enable_timing=True)
        model_end = torch.cuda.Event(enable_timing=True)

        with torch.no_grad():
            trace_start.record()
            depth = backend.trace(current_transform)
            trace_end.record()
            predictions = torch.sigmoid(model(depth.unsqueeze(1)))
            if predictions.shape != observed_radar.shape:
                raise RuntimeError(
                    f"Model output shape {tuple(predictions.shape)} does not match "
                    f"observed radar {tuple(observed_radar.shape)}."
                )
            model_end.record()
            residual = observed_radar - predictions
            cost_gpu = 0.5 * residual.square().sum()
            cost = float(cost_gpu.item())

    timing = {
        "depth_patch_generation_time_s": trace_start.elapsed_time(trace_end) / 1000.0,
        "model_inference_time_s": trace_end.elapsed_time(model_end) / 1000.0,
    }
    return cost, timing
