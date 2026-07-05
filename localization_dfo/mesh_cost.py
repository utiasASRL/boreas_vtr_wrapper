from dataclasses import dataclass, field
from time import perf_counter
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
import torch
from pylgmath.se3 import operations as se3op


@dataclass
class CostResult:
    cost: float
    diagnostics: List[Dict[str, Any]] = field(default_factory=list)


@dataclass
class GeometryParams:
    theta_min: float
    theta_max: float
    phi_min: float
    phi_max: float
    dtheta: float
    dphi: float
    width: int
    height: int
    max_uv_edge_length: Optional[float] = None
    max_depth_jump: Optional[float] = None
    fill_value: float = 0.0


@dataclass
class ResidualBuildOptions:
    azimuth_indices: Optional[Sequence[int]] = None
    device: Optional[str] = "cuda"
    dtype: Any = None
    model_output_activation: str = "raw"
    candidate_batch_size: int = 32
    geometry_batch_size: int = 16
    keep_diagnostics: bool = True


def _validate_geometry(geom: GeometryParams):
    if not np.isclose(geom.theta_min, -geom.theta_max):
        raise ValueError("torch_mesh expects symmetric theta bounds.")
    if not np.isclose(geom.phi_min, -geom.phi_max):
        raise ValueError("torch_mesh expects symmetric phi bounds.")
    if geom.theta_max <= 0.0 or geom.theta_max >= np.pi / 2:
        raise ValueError("theta_max must be in (0, pi/2).")
    if geom.phi_max <= 0.0 or geom.phi_max >= np.pi / 2:
        raise ValueError("phi_max must be in (0, pi/2).")


def _as_points_xyz(P_v, device):
    if torch.is_tensor(P_v):
        points = P_v
        if points.ndim == 3 and points.shape[1:] == (4, 1):
            points = points[:, :3, 0]
        elif points.ndim == 2 and points.shape[1] == 4:
            points = points[:, :3]
        elif points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"Unsupported point tensor shape: {tuple(points.shape)}")
        return points.to(device=device, dtype=torch.float32)

    points = np.asarray(P_v)
    if points.ndim == 3 and points.shape[1:] == (4, 1):
        points = points[:, :3, 0]
    elif points.ndim == 2 and points.shape[1] == 4:
        points = points[:, :3]
    elif points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"Unsupported point array shape: {points.shape}")
    return torch.as_tensor(points, device=device, dtype=torch.float32)


def _as_triangles(triangles, device, point_count):
    faces = torch.as_tensor(triangles, device=device, dtype=torch.long)
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"Mesh triangles must have shape (F, 3), got {tuple(faces.shape)}.")
    if faces.numel() == 0:
        raise ValueError("Mesh contains no triangles.")
    if int(faces.min().item()) < 0 or int(faces.max().item()) >= point_count:
        raise ValueError("Mesh triangles do not validly index vertices.")
    return faces


def _project_vertices(P_r, azimuths, geom: GeometryParams):
    c = torch.cos(azimuths)
    s = torch.sin(azimuths)
    px = P_r[:, 0]
    py = P_r[:, 1]
    pz = P_r[:, 2]

    xl = c * px + s * py
    yl = -s * px + c * py
    rho2 = xl * xl + yl * yl
    rho = torch.sqrt(torch.clamp(rho2, min=torch.finfo(P_r.dtype).eps))
    d = torch.sqrt(rho2 + pz * pz)
    theta = torch.atan2(yl, xl)
    phi = torch.atan2(pz, rho)

    I = torch.empty((P_r.shape[0], 3), device=P_r.device, dtype=P_r.dtype)
    I[:, 0] = d
    I[:, 1] = (theta - geom.theta_min) / geom.dtheta
    I[:, 2] = (geom.phi_max - phi) / geom.dphi
    return I, xl


def _empty_depth_batch(batch_size, geom: GeometryParams, device, dtype):
    return torch.full(
        (batch_size, int(geom.height), int(geom.width)),
        float(geom.fill_value),
        device=device,
        dtype=dtype,
    )


def _rasterize_faces(I_faces, face_batch_ids, batch_size, geom: GeometryParams, eps=1e-5):
    device = I_faces.device
    dtype = I_faces.dtype
    height = int(geom.height)
    width = int(geom.width)
    pixels_per_image = height * width
    raster_timing = {
        "raster_face_filter_time_s": 0.0,
        "raster_bbox_time_s": 0.0,
        "raster_pair_generation_time_s": 0.0,
        "raster_barycentric_time_s": 0.0,
        "raster_depth_interpolation_time_s": 0.0,
        "raster_zbuffer_time_s": 0.0,
        "raster_finalize_time_s": 0.0,
    }

    def mark_timing(key, start_time):
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        raster_timing[key] = perf_counter() - start_time

    def stats(valid_face_counts, triangle_pixel_test_counts, covered_pixel_counts):
        return {
            "valid_face_counts": valid_face_counts,
            "triangle_pixel_test_counts": triangle_pixel_test_counts,
            "covered_pixel_counts": covered_pixel_counts,
            "raster_timing": dict(raster_timing),
        }

    if I_faces.numel() == 0:
        zeros = torch.zeros(batch_size, device=device, dtype=torch.long)
        return _empty_depth_batch(batch_size, geom, device, dtype), stats(zeros, zeros, zeros)

    t_stage = perf_counter()
    uv = I_faces[:, :, 1:3]
    depths = I_faces[:, :, 0]
    finite = torch.isfinite(I_faces).all(dim=(1, 2))
    positive_depth = (depths > 0.0).all(dim=1)
    keep = finite & positive_depth

    if geom.max_depth_jump is not None:
        keep &= (depths.max(dim=1).values - depths.min(dim=1).values) <= float(geom.max_depth_jump)
    if geom.max_uv_edge_length is not None:
        edge_01 = torch.linalg.vector_norm(uv[:, 0] - uv[:, 1], dim=1)
        edge_12 = torch.linalg.vector_norm(uv[:, 1] - uv[:, 2], dim=1)
        edge_20 = torch.linalg.vector_norm(uv[:, 2] - uv[:, 0], dim=1)
        keep &= torch.maximum(torch.maximum(edge_01, edge_12), edge_20) <= float(geom.max_uv_edge_length)

    den_all = (
        (uv[:, 1, 1] - uv[:, 2, 1]) * (uv[:, 0, 0] - uv[:, 2, 0])
        + (uv[:, 2, 0] - uv[:, 1, 0]) * (uv[:, 0, 1] - uv[:, 2, 1])
    )
    keep &= torch.abs(den_all) > eps
    mark_timing("raster_face_filter_time_s", t_stage)

    if not torch.any(keep):
        zeros = torch.zeros(batch_size, device=device, dtype=torch.long)
        return _empty_depth_batch(batch_size, geom, device, dtype), stats(zeros, zeros, zeros)

    t_stage = perf_counter()
    I_faces = I_faces[keep]
    face_batch_ids = face_batch_ids[keep]
    uv = I_faces[:, :, 1:3]
    valid_face_counts = torch.bincount(face_batch_ids, minlength=batch_size)

    u_min = torch.floor(uv[:, :, 0].min(dim=1).values).to(torch.long)
    u_max = torch.ceil(uv[:, :, 0].max(dim=1).values).to(torch.long)
    v_min = torch.floor(uv[:, :, 1].min(dim=1).values).to(torch.long)
    v_max = torch.ceil(uv[:, :, 1].max(dim=1).values).to(torch.long)
    box_valid = (u_max >= 0) & (u_min < width) & (v_max >= 0) & (v_min < height)
    mark_timing("raster_bbox_time_s", t_stage)

    if not torch.any(box_valid):
        zeros = torch.zeros(batch_size, device=device, dtype=torch.long)
        return _empty_depth_batch(batch_size, geom, device, dtype), stats(valid_face_counts, zeros, zeros)

    t_stage = perf_counter()
    local_face_ids = torch.nonzero(box_valid, as_tuple=False).reshape(-1)
    u_min = torch.clamp(u_min[local_face_ids], 0, width - 1)
    u_max = torch.clamp(u_max[local_face_ids], 0, width - 1)
    v_min = torch.clamp(v_min[local_face_ids], 0, height - 1)
    v_max = torch.clamp(v_max[local_face_ids], 0, height - 1)
    box_width = u_max - u_min + 1
    box_height = v_max - v_min + 1
    pair_counts = box_width * box_height
    total_pairs = int(pair_counts.sum().item())

    if total_pairs == 0:
        zeros = torch.zeros(batch_size, device=device, dtype=torch.long)
        mark_timing("raster_pair_generation_time_s", t_stage)
        return _empty_depth_batch(batch_size, geom, device, dtype), stats(valid_face_counts, zeros, zeros)

    repeated_local_faces = torch.repeat_interleave(
        torch.arange(len(local_face_ids), device=device),
        pair_counts,
    )
    starts = torch.cumsum(pair_counts, dim=0) - pair_counts
    repeated_starts = torch.repeat_interleave(starts, pair_counts)
    pair_offsets = torch.arange(total_pairs, device=device) - repeated_starts
    repeated_width = box_width[repeated_local_faces]
    pixel_u = u_min[repeated_local_faces] + torch.remainder(pair_offsets, repeated_width)
    pixel_v = u_min.new_empty(total_pairs)
    pixel_v[:] = v_min[repeated_local_faces] + torch.div(
        pair_offsets,
        repeated_width,
        rounding_mode="floor",
    )

    face_ids = local_face_ids[repeated_local_faces]
    pair_faces = I_faces[face_ids]
    pair_uv = pair_faces[:, :, 1:3]
    qx = pixel_u.to(dtype)
    qy = pixel_v.to(dtype)
    mark_timing("raster_pair_generation_time_s", t_stage)

    t_stage = perf_counter()
    den = (
        (pair_uv[:, 1, 1] - pair_uv[:, 2, 1]) * (pair_uv[:, 0, 0] - pair_uv[:, 2, 0])
        + (pair_uv[:, 2, 0] - pair_uv[:, 1, 0]) * (pair_uv[:, 0, 1] - pair_uv[:, 2, 1])
    )
    w0 = (
        (pair_uv[:, 1, 1] - pair_uv[:, 2, 1]) * (qx - pair_uv[:, 2, 0])
        + (pair_uv[:, 2, 0] - pair_uv[:, 1, 0]) * (qy - pair_uv[:, 2, 1])
    ) / den
    w1 = (
        (pair_uv[:, 2, 1] - pair_uv[:, 0, 1]) * (qx - pair_uv[:, 2, 0])
        + (pair_uv[:, 0, 0] - pair_uv[:, 2, 0]) * (qy - pair_uv[:, 2, 1])
    ) / den
    w2 = 1.0 - w0 - w1
    weights = torch.stack((w0, w1, w2), dim=1)
    inside = (weights >= -eps).all(dim=1)
    mark_timing("raster_barycentric_time_s", t_stage)

    t_stage = perf_counter()
    depth_values = torch.sum(weights * pair_faces[:, :, 0], dim=1)
    valid_pairs = inside & torch.isfinite(depth_values) & (depth_values > 0.0)
    mark_timing("raster_depth_interpolation_time_s", t_stage)

    t_stage = perf_counter()
    D_flat = torch.full(
        (batch_size * pixels_per_image,),
        float("inf"),
        device=device,
        dtype=dtype,
    )
    global_pixels = face_batch_ids[face_ids] * pixels_per_image + pixel_v * width + pixel_u
    if torch.any(valid_pairs):
        D_flat.scatter_reduce_(
            0,
            global_pixels[valid_pairs],
            depth_values[valid_pairs],
            reduce="amin",
            include_self=True,
        )
    mark_timing("raster_zbuffer_time_s", t_stage)

    t_stage = perf_counter()
    covered = torch.isfinite(D_flat)
    D_flat = torch.where(covered, D_flat, torch.full_like(D_flat, float(geom.fill_value)))
    pair_batch_ids = face_batch_ids[face_ids]
    triangle_pixel_test_counts = torch.bincount(pair_batch_ids, minlength=batch_size)
    covered_pixel_counts = covered.reshape(batch_size, pixels_per_image).sum(dim=1)
    mark_timing("raster_finalize_time_s", t_stage)
    return D_flat.reshape(batch_size, height, width), stats(
        valid_face_counts,
        triangle_pixel_test_counts,
        covered_pixel_counts,
    )


def torch_mesh_depth_geometry_batch(
    P_v,
    triangles,
    T,
    odom_batch,
    radar_azimuth_batch,
    geom: GeometryParams,
    device,
):
    _validate_geometry(geom)
    device = torch.device(device)
    points = _as_points_xyz(P_v, device)
    faces = _as_triangles(triangles, device, len(points))
    T_batch_np = np.asarray([T @ odom_i for odom_i in odom_batch])
    T_batch = torch.as_tensor(T_batch_np, device=device, dtype=points.dtype)
    azimuths = torch.as_tensor(radar_azimuth_batch, device=device, dtype=points.dtype)
    batch_size = int(T_batch.shape[0])

    if device.type == "cuda":
        torch.cuda.synchronize(device)
    t0 = perf_counter()
    R = T_batch[:, :3, :3]
    t = T_batch[:, :3, 3]
    P_r_batch = torch.einsum("bij,nj->bni", R, points) + t[:, None, :]
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    transform_time = perf_counter() - t0

    t0 = perf_counter()
    cos_a = torch.cos(azimuths)[:, None]
    sin_a = torch.sin(azimuths)[:, None]
    x = P_r_batch[:, :, 0]
    y = P_r_batch[:, :, 1]
    z = P_r_batch[:, :, 2]
    x_local = cos_a * x + sin_a * y
    y_local = -sin_a * x + cos_a * y
    rho2 = x_local * x_local + y_local * y_local
    tan_theta = float(np.tan(geom.theta_max))
    tan_phi = float(np.tan(geom.phi_max))
    vertex_masks = (
        (x_local > 0.0)
        & (torch.abs(y_local) <= x_local * tan_theta)
        & (z * z <= rho2 * (tan_phi * tan_phi))
    )
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    frustum_time = perf_counter() - t0

    t0 = perf_counter()
    candidate_mask = vertex_masks[:, faces].any(dim=2)
    candidate_batch_ids, candidate_face_ids = torch.nonzero(candidate_mask, as_tuple=True)
    candidate_face_counts = torch.bincount(candidate_batch_ids, minlength=batch_size)
    selected_vertex_counts = vertex_masks.sum(dim=1)
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    face_selection_time = perf_counter() - t0

    t0 = perf_counter()
    if len(candidate_face_ids) == 0:
        I_faces = torch.empty((0, 3, 3), device=device, dtype=points.dtype)
        face_batch_ids = candidate_batch_ids
    else:
        candidate_vertex_ids = faces[candidate_face_ids]
        packed_vertex_keys = candidate_batch_ids[:, None] * len(points) + candidate_vertex_ids
        unique_vertex_keys, inverse = torch.unique(
            packed_vertex_keys.reshape(-1),
            sorted=False,
            return_inverse=True,
        )
        unique_batch_ids = torch.div(unique_vertex_keys, len(points), rounding_mode="floor")
        unique_vertex_ids = torch.remainder(unique_vertex_keys, len(points))
        P_r_unique = P_r_batch[unique_batch_ids, unique_vertex_ids]
        I_unique, x_local_unique = _project_vertices(
            P_r_unique,
            azimuths[unique_batch_ids],
            geom,
        )
        inverse = inverse.reshape(-1, 3)
        front_facing = (x_local_unique[inverse] > 0.0).all(dim=1)
        I_faces = I_unique[inverse][front_facing]
        face_batch_ids = candidate_batch_ids[front_facing]
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    selected_geometry_time = perf_counter() - t0

    t0 = perf_counter()
    D_batch, raster_stats = _rasterize_faces(I_faces, face_batch_ids, batch_size, geom)
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    rasterization_time = perf_counter() - t0

    t0 = perf_counter()
    D_batch_np = D_batch.detach().cpu().numpy()
    selected_vertex_counts_np = selected_vertex_counts.detach().cpu().numpy()
    candidate_face_counts_np = candidate_face_counts.detach().cpu().numpy()
    valid_face_counts_np = raster_stats["valid_face_counts"].detach().cpu().numpy()
    triangle_pixel_test_counts_np = raster_stats["triangle_pixel_test_counts"].detach().cpu().numpy()
    covered_pixel_counts_np = raster_stats["covered_pixel_counts"].detach().cpu().numpy()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    geometry_to_cpu_time = perf_counter() - t0

    outputs = []
    for batch_idx in range(batch_size):
        outputs.append((
            D_batch_np[batch_idx],
            None,
            {
                "selected_vertices": int(selected_vertex_counts_np[batch_idx]),
                "candidate_faces": int(candidate_face_counts_np[batch_idx]),
                "valid_faces": int(valid_face_counts_np[batch_idx]),
                "triangle_pixel_tests": int(triangle_pixel_test_counts_np[batch_idx]),
                "covered_pixels": int(covered_pixel_counts_np[batch_idx]),
            },
        ))

    timing_totals = {
        "point_transform_time_s": transform_time,
        "cartesian_frustum_mask_time_s": frustum_time,
        "candidate_face_selection_time_s": face_selection_time,
        "selected_geometry_time_s": selected_geometry_time,
        "mesh_rasterization_time_s": rasterization_time,
        "geometry_to_cpu_time_s": geometry_to_cpu_time,
    }
    timing_totals.update(raster_stats["raster_timing"])
    timing_per_azimuth = {
        key: value / max(batch_size, 1)
        for key, value in timing_totals.items()
    }
    timing_per_azimuth["pose_to_pixel_call_time_s"] = sum(timing_per_azimuth.values())
    return outputs, timing_per_azimuth


def _apply_model_output_activation(y, output_activation="raw"):
    if output_activation == "raw":
        return y
    if output_activation == "sigmoid":
        return torch.sigmoid(y)
    if output_activation == "softplus":
        return torch.nn.functional.softplus(y)
    raise ValueError(f"Unknown model output activation: {output_activation}")


def _model_forward_batch_numpy(model, D_batch_np, device=None, dtype=None, output_activation="raw"):
    model.eval()
    for param in model.parameters():
        param.requires_grad_(False)

    try:
        first_param = next(model.parameters())
    except StopIteration:
        first_param = None
    if device is None:
        device = first_param.device if first_param is not None else "cpu"
    device = torch.device(device)
    if dtype is None:
        dtype = first_param.dtype if first_param is not None else torch.float32
    model = model.to(device=device, dtype=dtype)

    D = torch.as_tensor(D_batch_np, dtype=dtype, device=device).unsqueeze(1)
    with torch.no_grad():
        m = _apply_model_output_activation(model(D), output_activation)
    return m.detach().cpu().numpy()


def _selected_azimuth_indices(odom_transforms, options: ResidualBuildOptions):
    if options.azimuth_indices is None:
        return list(range(len(odom_transforms)))
    return [int(i) for i in options.azimuth_indices]


def compute_radar_cost_only_fast(
    P_v,
    T,
    odom_transforms,
    m_obs_all,
    model,
    geom: GeometryParams,
    options: Optional[ResidualBuildOptions] = None,
    radar_azimuths=None,
    mesh_triangles=None,
) -> CostResult:
    options = ResidualBuildOptions() if options is None else options
    if mesh_triangles is None:
        raise ValueError("mesh_triangles is required.")

    azimuth_indices = _selected_azimuth_indices(odom_transforms, options)
    radar_azimuths = np.zeros(len(odom_transforms), dtype=float) if radar_azimuths is None else np.asarray(radar_azimuths).reshape(-1)
    model_batch_size = max(int(options.candidate_batch_size), 1)
    geometry_batch_size = max(int(options.geometry_batch_size), 1)

    cost = 0.0
    diagnostics = []
    patches = []
    obs_rows = []
    az_rows = []
    forward_times = []
    geometry_timings = []

    def flush_model_batch():
        nonlocal cost
        if not patches:
            return

        t_model = perf_counter()
        preds = _model_forward_batch_numpy(
            model,
            np.stack(patches).astype(np.float32, copy=False),
            device=options.device,
            dtype=options.dtype,
            output_activation=options.model_output_activation,
        )
        model_time_s = perf_counter() - t_model

        t_cost = perf_counter()
        patch_costs = []
        for obs, pred in zip(obs_rows, preds):
            e = np.asarray(obs).reshape(-1) - np.asarray(pred).reshape(-1)
            patch_cost = 0.5 * float(e @ e)
            cost += patch_cost
            patch_costs.append(patch_cost)
        cost_compute_time_s = perf_counter() - t_cost

        if options.keep_diagnostics:
            per_pred_model_time = model_time_s / max(len(preds), 1)
            per_pred_cost_time = cost_compute_time_s / max(len(preds), 1)
            for local_idx, patch_cost in enumerate(patch_costs):
                timing = dict(geometry_timings[local_idx])
                timing["cost_forward_time_s"] = forward_times[local_idx]
                timing["model_forward_time_s"] = per_pred_model_time
                timing["cost_compute_time_s"] = per_pred_cost_time
                diagnostics.append({
                    "azimuth": az_rows[local_idx],
                    "cost": patch_cost,
                    "timing": timing,
                })

        patches.clear()
        obs_rows.clear()
        az_rows.clear()
        forward_times.clear()
        geometry_timings.clear()

    def queue_depth(i, D, forward_time_s, timing):
        patches.append(D)
        obs_rows.append(np.asarray(m_obs_all[i]).reshape(-1))
        az_rows.append(i)
        forward_times.append(forward_time_s)
        geometry_timings.append(timing)
        if len(patches) >= model_batch_size:
            flush_model_batch()

    for start in range(0, len(azimuth_indices), geometry_batch_size):
        batch_indices = azimuth_indices[start:start + geometry_batch_size]
        t_forward = perf_counter()
        mesh_outputs, batch_timing = torch_mesh_depth_geometry_batch(
            P_v=P_v,
            triangles=mesh_triangles,
            T=T,
            odom_batch=odom_transforms[batch_indices],
            radar_azimuth_batch=radar_azimuths[batch_indices],
            geom=geom,
            device=options.device,
        )
        batch_wall_time_s = perf_counter() - t_forward

        for i, (D, _, mesh_stats) in zip(batch_indices, mesh_outputs):
            timing = dict(batch_timing)
            timing.update({
                "candidate_faces": float(mesh_stats["candidate_faces"]),
                "covered_pixels": float(mesh_stats["covered_pixels"]),
                "mesh_batch_wall_time_s": batch_wall_time_s / max(len(batch_indices), 1),
            })
            queue_depth(i, D, timing["mesh_batch_wall_time_s"], timing)

    flush_model_batch()
    return CostResult(cost=cost, diagnostics=diagnostics)


def left_se3_retract(T, delta):
    return se3op.vec2tran(np.asarray(delta).reshape(6, 1)) @ T
