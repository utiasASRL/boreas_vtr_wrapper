import numpy as np
import torch


class OptixDepthBackend:
    def __init__(self, geom, device):
        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise RuntimeError("The OptiX depth backend requires a CUDA device.")
        if self.device.index is None:
            self.device = torch.device("cuda", torch.cuda.current_device())
        if (int(geom.height), int(geom.width)) != (61, 61):
            raise ValueError(
                f"The radar translator CNN requires 61x61 patches, got {geom.height}x{geom.width}."
            )
        try:
            import optix_range_tracer
        except ImportError as error:
            raise RuntimeError(
                "Could not import optix_range_tracer. Enter through scripts/run_docker.sh "
                "and build the Stage 3 extension first."
            ) from error

        self.height = int(geom.height)
        self.width = int(geom.width)
        self.pose_count = 0
        self.mesh_ready = False
        self.scan_ready = False
        with torch.cuda.device(self.device):
            self.tracer = optix_range_tracer.OptixRangeTracer(
                width=self.width,
                height=self.height,
                theta_min=float(np.rad2deg(geom.theta_min)),
                theta_max=float(np.rad2deg(geom.theta_max)),
                phi_min=float(np.rad2deg(geom.phi_min)),
                phi_max=float(np.rad2deg(geom.phi_max)),
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


def _activate_model_output(predictions, activation):
    if activation == "raw":
        return predictions
    if activation == "sigmoid":
        return torch.sigmoid(predictions)
    if activation == "softplus":
        return torch.nn.functional.softplus(predictions)
    raise ValueError(f"Unknown model output activation: {activation}")


def compute_optix_cost(backend, current_transform, observed_radar, model, activation):
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

    trace_start = torch.cuda.Event(enable_timing=True)
    trace_end = torch.cuda.Event(enable_timing=True)
    model_end = torch.cuda.Event(enable_timing=True)
    cost_end = torch.cuda.Event(enable_timing=True)

    with torch.no_grad():
        trace_start.record()
        depth = backend.trace(current_transform)
        trace_end.record()
        predictions = _activate_model_output(model(depth.unsqueeze(1)), activation)
        if predictions.shape != observed_radar.shape:
            raise RuntimeError(
                f"Model output shape {tuple(predictions.shape)} does not match "
                f"observed radar {tuple(observed_radar.shape)}."
            )
        model_end.record()
        residual = observed_radar - predictions
        cost_gpu = 0.5 * residual.square().sum()
        cost_end.record()
        cost = float(cost_gpu.item())

    timing = {
        "optix_trace_time_s": trace_start.elapsed_time(trace_end) / 1000.0,
        "model_inference_time_s": trace_end.elapsed_time(model_end) / 1000.0,
        "cost_compute_time_s": model_end.elapsed_time(cost_end) / 1000.0,
    }
    return cost, depth, timing


def compare_depth_patches(depth_optix, depth_mesh):
    if tuple(depth_optix.shape) != tuple(depth_mesh.shape):
        raise ValueError(
            f"Depth patch shapes differ: OptiX {tuple(depth_optix.shape)} vs "
            f"torch-mesh {tuple(depth_mesh.shape)}."
        )

    hit_optix = depth_optix > 0
    hit_mesh = depth_mesh > 0
    both_hit = hit_optix & hit_mesh
    union = hit_optix | hit_mesh
    total = hit_optix.numel()
    agreement = (hit_optix == hit_mesh).sum().item() / max(total, 1)
    union_count = int(union.sum().item())
    intersection_count = int(both_hit.sum().item())

    metrics = {
        "total_pixels": total,
        "optix_hit_count": int(hit_optix.sum().item()),
        "torch_mesh_hit_count": int(hit_mesh.sum().item()),
        "hit_mask_agreement_percent": 100.0 * agreement,
        "hit_mask_iou": intersection_count / union_count if union_count else 1.0,
        "both_hit_count": intersection_count,
    }
    if intersection_count:
        error = (depth_optix[both_hit] - depth_mesh[both_hit]).abs()
        metrics.update(
            mean_absolute_error=float(error.mean().item()),
            median_absolute_error=float(error.median().item()),
            percentile_95_absolute_error=float(torch.quantile(error, 0.95).item()),
            maximum_absolute_error=float(error.max().item()),
        )
    else:
        metrics.update(
            mean_absolute_error=None,
            median_absolute_error=None,
            percentile_95_absolute_error=None,
            maximum_absolute_error=None,
        )

    def centroid(mask):
        indices = torch.nonzero(mask, as_tuple=False)
        if not len(indices):
            return None
        return tuple(float(value) for value in indices[:, 1:].float().mean(dim=0).tolist())

    optix_centroid = centroid(hit_optix)
    mesh_centroid = centroid(hit_mesh)
    metrics["optix_hit_centroid_row_col"] = optix_centroid
    metrics["torch_mesh_hit_centroid_row_col"] = mesh_centroid
    metrics["shape_consistent"] = True
    metrics["orientation_appears_consistent"] = (
        optix_centroid is not None
        and mesh_centroid is not None
        and max(abs(a - b) for a, b in zip(optix_centroid, mesh_centroid)) <= 2.0
    )
    return metrics


def print_comparison_metrics(metrics):
    print("Depth backend comparison")
    for key, value in metrics.items():
        print(f"  {key}: {value}")
