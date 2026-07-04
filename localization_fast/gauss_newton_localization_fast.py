from __future__ import annotations

import copy
from dataclasses import dataclass, field
from time import perf_counter
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np


Array = np.ndarray


@dataclass
class GNResult:
    state: Any
    history: List[Dict[str, Any]]


@dataclass
class LinearizationResult:
    e: Optional[Array]
    J: Optional[Array]
    H: Array
    g: Array
    cost: float
    diagnostics: List[Dict[str, Any]] = field(default_factory=list)


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
    mode: str = "stack"
    device: Optional[str] = "cuda"
    dtype: Any = None
    model_output_activation: str = "raw"
    active_dims: Optional[Sequence[int]] = None
    linearization_batch_size: int = 32
    candidate_batch_size: int = 32
    geometry_backend: str = "numpy_spherical"
    geometry_batch_size: int = 16
    keep_diagnostics: bool = True


def normalize_active_dims(active_dims: Optional[Sequence[int]]) -> List[int]:
    if active_dims is None:
        return list(range(6))
    dims = [int(dim) for dim in active_dims]
    if len(dims) == 0:
        raise ValueError("active_dims must contain at least one dimension.")
    if any(dim < 0 or dim >= 6 for dim in dims):
        raise ValueError(f"active_dims must be in [0, 5], got {dims}.")
    return dims


def validate_symmetric_geometry(geom: GeometryParams, atol: float = 1e-12) -> None:
    if not np.isclose(geom.theta_min, -geom.theta_max, atol=atol, rtol=0.0):
        raise ValueError("torch_frustum requires symmetric theta bounds.")
    if not np.isclose(geom.phi_min, -geom.phi_max, atol=atol, rtol=0.0):
        raise ValueError("torch_frustum requires symmetric phi bounds.")
    if geom.theta_max <= 0.0 or geom.theta_max >= 0.5 * np.pi:
        raise ValueError("torch_frustum requires 0 < theta_max < pi/2.")
    if geom.phi_max <= 0.0 or geom.phi_max >= 0.5 * np.pi:
        raise ValueError("torch_frustum requires 0 < phi_max < pi/2.")


def map_points_xyz_torch(P_v, device):
    import torch

    if torch.is_tensor(P_v):
        points = P_v
        if points.ndim == 3 and points.shape[-2:] == (4, 1):
            points = points[:, :3, 0]
        elif points.ndim == 2 and points.shape[1] == 4:
            points = points[:, :3]
        elif points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"Unsupported Torch point shape: {tuple(points.shape)}")
        return points.to(device=device, dtype=torch.float32)

    points_np = np.asarray(P_v)
    if points_np.ndim == 3 and points_np.shape[-2:] == (4, 1):
        points_np = points_np[:, :3, 0]
    elif points_np.ndim == 2 and points_np.shape[1] == 4:
        points_np = points_np[:, :3]
    elif points_np.ndim != 2 or points_np.shape[1] != 3:
        raise ValueError(f"Unsupported NumPy point shape: {points_np.shape}")
    return torch.as_tensor(points_np, device=device, dtype=torch.float32)


def torch_frustum_pixel_geometry_batch(
    P_v,
    T: Array,
    odom_batch: Array,
    radar_azimuth_batch: Array,
    geom: GeometryParams,
    device,
    with_jacobian: bool,
):
    import torch

    validate_symmetric_geometry(geom)
    device = torch.device(device)
    points = map_points_xyz_torch(P_v, device)
    T_batch_np = np.asarray([T @ odom_i for odom_i in odom_batch])
    T_batch = torch.as_tensor(T_batch_np, device=device, dtype=points.dtype)
    azimuths = torch.as_tensor(radar_azimuth_batch, device=device, dtype=points.dtype)

    use_cuda_timing = device.type == "cuda"
    events = []

    def mark():
        if use_cuda_timing:
            event = torch.cuda.Event(enable_timing=True)
            event.record()
            events.append(event)
        else:
            events.append(perf_counter())

    mark()
    R = T_batch[:, :3, :3]
    t = T_batch[:, :3, 3]
    P_r_batch = torch.einsum("bij,nj->bni", R, points) + t[:, None, :]
    mark()

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
    masks = (
        (x_local > 0.0)
        & (torch.abs(y_local) <= x_local * tan_theta)
        & (z * z <= rho2 * (tan_phi * tan_phi))
    )
    mark()

    batch_ids, point_ids = torch.nonzero(masks, as_tuple=True)
    selected_counts = torch.bincount(batch_ids, minlength=P_r_batch.shape[0])
    P_r = P_r_batch[batch_ids, point_ids]
    xl = x_local[batch_ids, point_ids]
    yl = y_local[batch_ids, point_ids]
    zl = z[batch_ids, point_ids]

    rho2_kept = xl * xl + yl * yl
    rho = torch.sqrt(rho2_kept)
    d2 = rho2_kept + zl * zl
    d = torch.sqrt(d2)
    theta = torch.atan2(yl, xl)
    phi = torch.atan2(zl, rho)

    M = P_r.shape[0]
    I = torch.empty((M, 3), device=device, dtype=points.dtype)
    I[:, 0] = d
    I[:, 1] = (theta - geom.theta_min) / geom.dtheta
    I[:, 2] = (geom.phi_max - phi) / geom.dphi

    J_I = None
    if with_jacobian:
        px = P_r[:, 0]
        py = P_r[:, 1]
        pz = P_r[:, 2]
        c = cos_a[:, 0][batch_ids]
        s = sin_a[:, 0][batch_ids]

        Jx = torch.zeros((M, 6), device=device, dtype=points.dtype)
        Jy = torch.zeros((M, 6), device=device, dtype=points.dtype)
        Jz = torch.zeros((M, 6), device=device, dtype=points.dtype)

        Jx[:, 0] = c
        Jx[:, 1] = s
        Jx[:, 3] = -s * pz
        Jx[:, 4] = c * pz
        Jx[:, 5] = -c * py + s * px

        Jy[:, 0] = -s
        Jy[:, 1] = c
        Jy[:, 3] = -c * pz
        Jy[:, 4] = -s * pz
        Jy[:, 5] = s * py + c * px

        Jz[:, 2] = 1.0
        Jz[:, 3] = py
        Jz[:, 4] = -px

        eps = torch.finfo(points.dtype).eps
        rho2_safe = torch.clamp(rho2_kept, min=eps)
        rho_safe = torch.clamp(rho, min=eps)
        d_safe = torch.clamp(d, min=eps)
        d2_safe = torch.clamp(d2, min=eps)

        J_I = torch.empty((M, 3, 6), device=device, dtype=points.dtype)
        J_I[:, 0, :] = (
            (xl / d_safe)[:, None] * Jx
            + (yl / d_safe)[:, None] * Jy
            + (zl / d_safe)[:, None] * Jz
        )
        J_I[:, 1, :] = (
            (-yl / rho2_safe)[:, None] * Jx
            + (xl / rho2_safe)[:, None] * Jy
        ) / geom.dtheta
        J_I[:, 2, :] = -(
            (-xl * zl / (d2_safe * rho_safe))[:, None] * Jx
            + (-yl * zl / (d2_safe * rho_safe))[:, None] * Jy
            + (rho_safe / d2_safe)[:, None] * Jz
        ) / geom.dphi
    mark()

    if J_I is None:
        packed = I
    else:
        packed = torch.cat((I, J_I.reshape(M, 18)), dim=1)
    packed_np = packed.detach().cpu().numpy()
    selected_counts_np = selected_counts.detach().cpu().numpy()
    mark()

    if use_cuda_timing:
        torch.cuda.synchronize(device)
        elapsed = [events[i].elapsed_time(events[i + 1]) * 1e-3 for i in range(len(events) - 1)]
    else:
        elapsed = [events[i + 1] - events[i] for i in range(len(events) - 1)]

    cpu_outputs = []
    offset = 0
    for selected_count in selected_counts_np:
        selected_count = int(selected_count)
        selected = packed_np[offset:offset + selected_count]
        I_np = selected[:, :3, None]
        J_np = None if J_I is None else selected[:, 3:].reshape(selected_count, 3, 6)
        cpu_outputs.append((I_np, J_np, selected_count))
        offset += selected_count

    batch_size = max(len(cpu_outputs), 1)
    timing_per_azimuth = {
        "point_transform_time_s": elapsed[0] / batch_size,
        "cartesian_frustum_mask_time_s": elapsed[1] / batch_size,
        "selected_geometry_time_s": elapsed[2] / batch_size,
        "geometry_to_cpu_time_s": elapsed[3] / batch_size,
        "pose_to_pixel_call_time_s": sum(elapsed) / batch_size,
    }
    return cpu_outputs, timing_per_azimuth


def map_mesh_triangles_torch(triangles, device):
    import torch

    faces = torch.as_tensor(triangles, device=device, dtype=torch.long)
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"Mesh triangles must have shape (F, 3), got {tuple(faces.shape)}.")
    if faces.numel() == 0:
        raise ValueError("Mesh contains no triangles.")
    return faces


def _project_mesh_face_vertices(
    P_r_faces,
    azimuth,
    geom: GeometryParams,
    active_dims,
    with_jacobian,
):
    """Project selected radar-frame face vertices to [depth, u, v]."""
    import torch

    c = torch.cos(azimuth)
    s = torch.sin(azimuth)
    px = P_r_faces[..., 0]
    py = P_r_faces[..., 1]
    pz = P_r_faces[..., 2]
    xl = c * px + s * py
    yl = -s * px + c * py
    zl = pz

    rho2 = xl * xl + yl * yl
    rho = torch.sqrt(rho2)
    d2 = rho2 + zl * zl
    d = torch.sqrt(d2)
    theta = torch.atan2(yl, xl)
    phi = torch.atan2(zl, rho)

    I_faces = torch.stack(
        (
            d,
            (theta - geom.theta_min) / geom.dtheta,
            (geom.phi_max - phi) / geom.dphi,
        ),
        dim=2,
    )
    if not with_jacobian:
        return I_faces, None, xl

    shape = (*px.shape, 6)
    Jx = torch.zeros(shape, device=px.device, dtype=px.dtype)
    Jy = torch.zeros_like(Jx)
    Jz = torch.zeros_like(Jx)

    Jx[..., 0] = c
    Jx[..., 1] = s
    Jx[..., 3] = -s * pz
    Jx[..., 4] = c * pz
    Jx[..., 5] = -c * py + s * px

    Jy[..., 0] = -s
    Jy[..., 1] = c
    Jy[..., 3] = -c * pz
    Jy[..., 4] = -s * pz
    Jy[..., 5] = s * py + c * px

    Jz[..., 2] = 1.0
    Jz[..., 3] = py
    Jz[..., 4] = -px

    eps = torch.finfo(px.dtype).eps
    rho2_safe = torch.clamp(rho2, min=eps)
    rho_safe = torch.clamp(rho, min=eps)
    d_safe = torch.clamp(d, min=eps)
    d2_safe = torch.clamp(d2, min=eps)

    J_I = torch.empty((*px.shape, 3, 6), device=px.device, dtype=px.dtype)
    J_I[..., 0, :] = (
        (xl / d_safe)[..., None] * Jx
        + (yl / d_safe)[..., None] * Jy
        + (zl / d_safe)[..., None] * Jz
    )
    J_I[..., 1, :] = (
        (-yl / rho2_safe)[..., None] * Jx
        + (xl / rho2_safe)[..., None] * Jy
    ) / geom.dtheta
    J_I[..., 2, :] = -(
        (-xl * zl / (d2_safe * rho_safe))[..., None] * Jx
        + (-yl * zl / (d2_safe * rho_safe))[..., None] * Jy
        + (rho_safe / d2_safe)[..., None] * Jz
    ) / geom.dphi
    return I_faces, J_I[..., list(active_dims)], xl


def _project_packed_mesh_vertices(
    P_r,
    azimuths,
    geom: GeometryParams,
    active_dims,
    with_jacobian,
):
    """Project packed vertices whose azimuths may differ per row."""
    import torch

    c = torch.cos(azimuths)
    s = torch.sin(azimuths)
    px = P_r[:, 0]
    py = P_r[:, 1]
    pz = P_r[:, 2]
    xl = c * px + s * py
    yl = -s * px + c * py
    zl = pz

    rho2 = xl * xl + yl * yl
    rho = torch.sqrt(rho2)
    d2 = rho2 + zl * zl
    d = torch.sqrt(d2)
    theta = torch.atan2(yl, xl)
    phi = torch.atan2(zl, rho)
    I = torch.stack(
        (
            d,
            (theta - geom.theta_min) / geom.dtheta,
            (geom.phi_max - phi) / geom.dphi,
        ),
        dim=1,
    )
    if not with_jacobian:
        return I, None, xl

    M = len(P_r)
    Jx = torch.zeros((M, 6), device=P_r.device, dtype=P_r.dtype)
    Jy = torch.zeros_like(Jx)
    Jz = torch.zeros_like(Jx)

    Jx[:, 0] = c
    Jx[:, 1] = s
    Jx[:, 3] = -s * pz
    Jx[:, 4] = c * pz
    Jx[:, 5] = -c * py + s * px

    Jy[:, 0] = -s
    Jy[:, 1] = c
    Jy[:, 3] = -c * pz
    Jy[:, 4] = -s * pz
    Jy[:, 5] = s * py + c * px

    Jz[:, 2] = 1.0
    Jz[:, 3] = py
    Jz[:, 4] = -px

    eps = torch.finfo(P_r.dtype).eps
    rho2_safe = torch.clamp(rho2, min=eps)
    rho_safe = torch.clamp(rho, min=eps)
    d_safe = torch.clamp(d, min=eps)
    d2_safe = torch.clamp(d2, min=eps)

    J_I = torch.empty((M, 3, 6), device=P_r.device, dtype=P_r.dtype)
    J_I[:, 0, :] = (
        (xl / d_safe)[:, None] * Jx
        + (yl / d_safe)[:, None] * Jy
        + (zl / d_safe)[:, None] * Jz
    )
    J_I[:, 1, :] = (
        (-yl / rho2_safe)[:, None] * Jx
        + (xl / rho2_safe)[:, None] * Jy
    ) / geom.dtheta
    J_I[:, 2, :] = -(
        (-xl * zl / (d2_safe * rho_safe))[:, None] * Jx
        + (-yl * zl / (d2_safe * rho_safe))[:, None] * Jy
        + (rho_safe / d2_safe)[:, None] * Jz
    ) / geom.dphi
    return I, J_I[:, :, list(active_dims)], xl


def _rasterize_packed_mesh_faces_torch(
    I_faces,
    J_I_faces,
    face_batch_ids,
    batch_size,
    geom: GeometryParams,
    barycentric_epsilon=1e-5,
):
    """Rasterize packed faces from several azimuths with one z-buffer reduction."""
    import torch

    device = I_faces.device
    dtype = I_faces.dtype
    height = int(geom.height)
    width = int(geom.width)
    pixels_per_image = height * width
    total_pixels = batch_size * pixels_per_image
    active_dim_count = 0 if J_I_faces is None else J_I_faces.shape[-1]

    finite = torch.isfinite(I_faces).all(dim=(1, 2))
    positive_depth = (I_faces[:, :, 0] > 0.0).all(dim=1)
    keep = finite & positive_depth

    depths = I_faces[:, :, 0]
    if geom.max_depth_jump is not None:
        keep &= (depths.max(dim=1).values - depths.min(dim=1).values) <= float(
            geom.max_depth_jump
        )

    uv = I_faces[:, :, 1:3]
    if geom.max_uv_edge_length is not None:
        edge_01 = torch.linalg.vector_norm(uv[:, 0] - uv[:, 1], dim=1)
        edge_12 = torch.linalg.vector_norm(uv[:, 1] - uv[:, 2], dim=1)
        edge_20 = torch.linalg.vector_norm(uv[:, 2] - uv[:, 0], dim=1)
        keep &= torch.maximum(torch.maximum(edge_01, edge_12), edge_20) <= float(
            geom.max_uv_edge_length
        )

    denominator = (
        (uv[:, 1, 1] - uv[:, 2, 1]) * (uv[:, 0, 0] - uv[:, 2, 0])
        + (uv[:, 2, 0] - uv[:, 1, 0]) * (uv[:, 0, 1] - uv[:, 2, 1])
    )
    keep &= torch.abs(denominator) > 1e-8

    u_min = torch.ceil(uv[:, :, 0].min(dim=1).values).to(torch.long)
    u_max = torch.floor(uv[:, :, 0].max(dim=1).values).to(torch.long)
    v_min = torch.ceil(uv[:, :, 1].min(dim=1).values).to(torch.long)
    v_max = torch.floor(uv[:, :, 1].max(dim=1).values).to(torch.long)

    # A face is useful only if its projected bounding box intersects the image
    # and contains at least one integer pixel-center coordinate.
    keep &= (
        (u_min <= u_max)
        & (v_min <= v_max)
        & (u_max >= 0)
        & (u_min <= width - 1)
        & (v_max >= 0)
        & (v_min <= height - 1)
    )

    I_faces = I_faces[keep]
    face_batch_ids = face_batch_ids[keep]
    u_min = torch.clamp(u_min[keep], 0, width - 1)
    u_max = torch.clamp(u_max[keep], 0, width - 1)
    v_min = torch.clamp(v_min[keep], 0, height - 1)
    v_max = torch.clamp(v_max[keep], 0, height - 1)
    if J_I_faces is not None:
        J_I_faces = J_I_faces[keep]

    valid_face_counts = torch.bincount(face_batch_ids, minlength=batch_size)
    if len(I_faces) == 0:
        D = torch.full(
            (batch_size, height, width),
            float(geom.fill_value),
            device=device,
            dtype=dtype,
        )
        J_D = None
        if J_I_faces is not None:
            J_D = torch.zeros(
                (batch_size, pixels_per_image, active_dim_count),
                device=device,
                dtype=dtype,
            )
        zeros = torch.zeros(batch_size, device=device, dtype=torch.long)
        return D, J_D, {
            "valid_face_counts": valid_face_counts,
            "triangle_pixel_test_counts": zeros,
            "covered_pixel_counts": zeros,
        }

    box_width = u_max - u_min + 1
    box_height = v_max - v_min + 1
    pair_counts = box_width * box_height
    total_pairs = int(pair_counts.sum().item())
    repeated_faces = torch.repeat_interleave(
        torch.arange(len(I_faces), device=device),
        pair_counts,
    )
    starts = torch.cumsum(pair_counts, dim=0) - pair_counts
    repeated_starts = torch.repeat_interleave(starts, pair_counts)
    pair_offsets = torch.arange(total_pairs, device=device) - repeated_starts
    repeated_width = box_width[repeated_faces]
    pixel_u = u_min[repeated_faces] + torch.remainder(pair_offsets, repeated_width)
    pixel_v = v_min[repeated_faces] + torch.div(
        pair_offsets,
        repeated_width,
        rounding_mode="floor",
    )

    pair_faces = I_faces[repeated_faces]
    pair_uv = pair_faces[:, :, 1:3]
    qx = pixel_u.to(dtype)
    qy = pixel_v.to(dtype)
    den = (
        (pair_uv[:, 1, 1] - pair_uv[:, 2, 1])
        * (pair_uv[:, 0, 0] - pair_uv[:, 2, 0])
        + (pair_uv[:, 2, 0] - pair_uv[:, 1, 0])
        * (pair_uv[:, 0, 1] - pair_uv[:, 2, 1])
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
    inside = (weights >= -barycentric_epsilon).all(dim=1)

    pair_face_ids = repeated_faces[inside]
    weights = weights[inside]
    pixel_u = pixel_u[inside]
    pixel_v = pixel_v[inside]
    pair_batch_ids = face_batch_ids[pair_face_ids]
    local_pixel_ids = pixel_v * width + pixel_u
    global_pixel_ids = pair_batch_ids * pixels_per_image + local_pixel_ids
    pair_depths = torch.sum(weights * I_faces[pair_face_ids, :, 0], dim=1)

    z_buffer = torch.full((total_pixels,), torch.inf, device=device, dtype=dtype)
    z_buffer.scatter_reduce_(
        0,
        global_pixel_ids,
        pair_depths,
        reduce="amin",
        include_self=True,
    )
    covered = torch.isfinite(z_buffer)
    D_flat = torch.full(
        (total_pixels,),
        float(geom.fill_value),
        device=device,
        dtype=dtype,
    )
    D_flat[covered] = z_buffer[covered]

    J_D = None
    if J_I_faces is not None:
        J_D_flat = torch.zeros(
            (total_pixels, active_dim_count),
            device=device,
            dtype=dtype,
        )
        is_winner = torch.abs(pair_depths - z_buffer[global_pixel_ids]) <= 1e-6
        pair_indices = torch.arange(len(pair_depths), device=device, dtype=torch.long)
        sentinel = torch.full_like(pair_indices, len(pair_depths))
        winner_candidates = torch.where(is_winner, pair_indices, sentinel)
        winner_pair = torch.full(
            (total_pixels,),
            len(pair_depths),
            device=device,
            dtype=torch.long,
        )
        winner_pair.scatter_reduce_(
            0,
            global_pixel_ids,
            winner_candidates,
            reduce="amin",
            include_self=True,
        )
        covered_pixels = torch.nonzero(covered, as_tuple=False).reshape(-1)
        selected_pairs = winner_pair[covered_pixels]
        valid_selected = selected_pairs < len(pair_depths)
        covered_pixels = covered_pixels[valid_selected]
        selected_pairs = selected_pairs[valid_selected]

        selected_faces = pair_face_ids[selected_pairs]
        selected_weights = weights[selected_pairs]
        face_I = I_faces[selected_faces]
        face_J = J_I_faces[selected_faces]

        A = torch.ones((len(covered_pixels), 3, 3), device=device, dtype=dtype)
        A[:, 0, :] = face_I[:, :, 1]
        A[:, 1, :] = face_I[:, :, 2]
        A_inv = torch.linalg.inv(A)

        du = face_J[:, :, 1, :]
        dv = face_J[:, :, 2, :]
        rhs = torch.zeros(
            (len(covered_pixels), 3, active_dim_count),
            device=device,
            dtype=dtype,
        )
        rhs[:, 0, :] = torch.sum(du * selected_weights[:, :, None], dim=1)
        rhs[:, 1, :] = torch.sum(dv * selected_weights[:, :, None], dim=1)
        dweights = -torch.matmul(A_inv, rhs)

        depth_J = face_J[:, :, 0, :]
        vertex_depths = face_I[:, :, 0]
        J_D_flat[covered_pixels] = (
            torch.sum(selected_weights[:, :, None] * depth_J, dim=1)
            + torch.sum(vertex_depths[:, :, None] * dweights, dim=1)
        )
        J_D = J_D_flat.reshape(batch_size, pixels_per_image, active_dim_count)

    triangle_pixel_test_counts = torch.bincount(
        torch.repeat_interleave(face_batch_ids, pair_counts),
        minlength=batch_size,
    )
    covered_pixel_counts = covered.reshape(batch_size, pixels_per_image).sum(dim=1)
    return D_flat.reshape(batch_size, height, width), J_D, {
        "valid_face_counts": valid_face_counts,
        "triangle_pixel_test_counts": triangle_pixel_test_counts,
        "covered_pixel_counts": covered_pixel_counts,
    }


def _rasterize_projected_mesh_faces_torch(
    I_faces,
    J_I_faces,
    geom: GeometryParams,
    face_chunk_size=512,
    barycentric_epsilon=1e-5,
):
    """Rasterize fixed projected faces at integer pixel-center coordinates."""
    import torch

    device = I_faces.device
    dtype = I_faces.dtype
    height = int(geom.height)
    width = int(geom.width)
    num_pixels = height * width
    active_dim_count = 0 if J_I_faces is None else J_I_faces.shape[-1]

    finite = torch.isfinite(I_faces).all(dim=(1, 2))
    positive_depth = (I_faces[:, :, 0] > 0.0).all(dim=1)
    keep = finite & positive_depth

    if geom.max_depth_jump is not None:
        depths = I_faces[:, :, 0]
        keep &= (depths.max(dim=1).values - depths.min(dim=1).values) <= float(
            geom.max_depth_jump
        )

    uv = I_faces[:, :, 1:3]
    if geom.max_uv_edge_length is not None:
        edge_01 = torch.linalg.vector_norm(uv[:, 0] - uv[:, 1], dim=1)
        edge_12 = torch.linalg.vector_norm(uv[:, 1] - uv[:, 2], dim=1)
        edge_20 = torch.linalg.vector_norm(uv[:, 2] - uv[:, 0], dim=1)
        keep &= torch.maximum(torch.maximum(edge_01, edge_12), edge_20) <= float(
            geom.max_uv_edge_length
        )

    denominator = (
        (uv[:, 1, 1] - uv[:, 2, 1]) * (uv[:, 0, 0] - uv[:, 2, 0])
        + (uv[:, 2, 0] - uv[:, 1, 0]) * (uv[:, 0, 1] - uv[:, 2, 1])
    )
    keep &= torch.abs(denominator) > 1e-8

    uv_min = uv.min(dim=1).values
    uv_max = uv.max(dim=1).values
    keep &= (
        (uv_max[:, 0] >= 0.0)
        & (uv_min[:, 0] <= width - 1)
        & (uv_max[:, 1] >= 0.0)
        & (uv_min[:, 1] <= height - 1)
    )

    I_faces = I_faces[keep]
    if J_I_faces is not None:
        J_I_faces = J_I_faces[keep]
    if I_faces.shape[0] == 0:
        D = torch.full((height, width), float(geom.fill_value), device=device, dtype=dtype)
        J_D = None
        if J_I_faces is not None:
            J_D = torch.zeros((num_pixels, active_dim_count), device=device, dtype=dtype)
        return D, J_D, {
            "valid_faces": 0,
            "triangle_pixel_tests": 0,
            "covered_pixels": 0,
        }

    z_buffer = torch.full((num_pixels,), torch.inf, device=device, dtype=dtype)
    winner_face = torch.full((num_pixels,), -1, device=device, dtype=torch.long)
    winner_weights = torch.zeros((num_pixels, 3), device=device, dtype=dtype)
    triangle_pixel_tests = 0

    face_chunk_size = max(int(face_chunk_size), 1)
    for start in range(0, I_faces.shape[0], face_chunk_size):
        stop = min(start + face_chunk_size, I_faces.shape[0])
        chunk = I_faces[start:stop]
        chunk_uv = chunk[:, :, 1:3]

        u_min = torch.ceil(chunk_uv[:, :, 0].min(dim=1).values).to(torch.long)
        u_max = torch.floor(chunk_uv[:, :, 0].max(dim=1).values).to(torch.long)
        v_min = torch.ceil(chunk_uv[:, :, 1].min(dim=1).values).to(torch.long)
        v_max = torch.floor(chunk_uv[:, :, 1].max(dim=1).values).to(torch.long)

        box_valid = (
            (u_max >= 0)
            & (u_min <= width - 1)
            & (v_max >= 0)
            & (v_min <= height - 1)
        )
        if not torch.any(box_valid):
            continue

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
            continue
        triangle_pixel_tests += total_pairs

        repeated_local_faces = torch.repeat_interleave(
            torch.arange(len(local_face_ids), device=device),
            pair_counts,
        )
        starts = torch.cumsum(pair_counts, dim=0) - pair_counts
        repeated_starts = torch.repeat_interleave(starts, pair_counts)
        pair_offsets = torch.arange(total_pairs, device=device) - repeated_starts
        repeated_width = box_width[repeated_local_faces]
        pixel_u = u_min[repeated_local_faces] + torch.remainder(
            pair_offsets,
            repeated_width,
        )
        pixel_v = v_min[repeated_local_faces] + torch.div(
            pair_offsets,
            repeated_width,
            rounding_mode="floor",
        )

        chunk_face_ids = local_face_ids[repeated_local_faces]
        pair_faces = chunk[chunk_face_ids]
        pair_uv = pair_faces[:, :, 1:3]
        qx = pixel_u.to(dtype)
        qy = pixel_v.to(dtype)

        den = (
            (pair_uv[:, 1, 1] - pair_uv[:, 2, 1])
            * (pair_uv[:, 0, 0] - pair_uv[:, 2, 0])
            + (pair_uv[:, 2, 0] - pair_uv[:, 1, 0])
            * (pair_uv[:, 0, 1] - pair_uv[:, 2, 1])
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
        inside = (weights >= -barycentric_epsilon).all(dim=1)
        if not torch.any(inside):
            continue

        pixel_ids = pixel_v[inside] * width + pixel_u[inside]
        weights = weights[inside]
        global_face_ids = start + chunk_face_ids[inside]
        depths = torch.sum(weights * pair_faces[inside, :, 0], dim=1)

        chunk_z = torch.full_like(z_buffer, torch.inf)
        chunk_z.scatter_reduce_(
            0,
            pixel_ids,
            depths,
            reduce="amin",
            include_self=True,
        )
        improves = chunk_z < z_buffer
        if not torch.any(improves):
            continue

        # Find one pair responsible for each improved minimum. Exact ties are
        # resolved by the earliest pair, which gives deterministic active sets.
        is_chunk_winner = torch.abs(depths - chunk_z[pixel_ids]) <= 1e-6
        pair_indices = torch.arange(len(depths), device=device, dtype=torch.long)
        sentinel = torch.full_like(pair_indices, len(depths))
        winning_pair_candidates = torch.where(is_chunk_winner, pair_indices, sentinel)
        winning_pair = torch.full(
            (num_pixels,),
            len(depths),
            device=device,
            dtype=torch.long,
        )
        winning_pair.scatter_reduce_(
            0,
            pixel_ids,
            winning_pair_candidates,
            reduce="amin",
            include_self=True,
        )
        improved_pixels = torch.nonzero(improves, as_tuple=False).reshape(-1)
        selected_pairs = winning_pair[improved_pixels]
        valid_selected = selected_pairs < len(depths)
        improved_pixels = improved_pixels[valid_selected]
        selected_pairs = selected_pairs[valid_selected]
        z_buffer[improved_pixels] = chunk_z[improved_pixels]
        winner_face[improved_pixels] = global_face_ids[selected_pairs]
        winner_weights[improved_pixels] = weights[selected_pairs]

    covered = torch.isfinite(z_buffer)
    D_flat = torch.full(
        (num_pixels,),
        float(geom.fill_value),
        device=device,
        dtype=dtype,
    )
    D_flat[covered] = z_buffer[covered]

    J_D = None
    if J_I_faces is not None:
        J_D = torch.zeros((num_pixels, active_dim_count), device=device, dtype=dtype)
        pixel_ids = torch.nonzero(covered, as_tuple=False).reshape(-1)
        faces = winner_face[pixel_ids]
        weights = winner_weights[pixel_ids]
        face_I = I_faces[faces]
        face_J = J_I_faces[faces]

        # w = A^-1 q, with q fixed at the integer pixel center.
        A = torch.ones((len(pixel_ids), 3, 3), device=device, dtype=dtype)
        A[:, 0, :] = face_I[:, :, 1]
        A[:, 1, :] = face_I[:, :, 2]
        A_inv = torch.linalg.inv(A)

        du = face_J[:, :, 1, :]
        dv = face_J[:, :, 2, :]
        rhs = torch.zeros(
            (len(pixel_ids), 3, active_dim_count),
            device=device,
            dtype=dtype,
        )
        rhs[:, 0, :] = torch.sum(du * weights[:, :, None], dim=1)
        rhs[:, 1, :] = torch.sum(dv * weights[:, :, None], dim=1)
        dweights = -torch.matmul(A_inv, rhs)

        depth_J = face_J[:, :, 0, :]
        vertex_depths = face_I[:, :, 0]
        J_values = (
            torch.sum(weights[:, :, None] * depth_J, dim=1)
            + torch.sum(vertex_depths[:, :, None] * dweights, dim=1)
        )
        J_D[pixel_ids] = J_values

    return D_flat.reshape(height, width), J_D, {
        "valid_faces": int(I_faces.shape[0]),
        "triangle_pixel_tests": triangle_pixel_tests,
        "covered_pixels": int(covered.sum().item()),
    }


def torch_mesh_depth_geometry_batch(
    P_v,
    triangles,
    T: Array,
    odom_batch: Array,
    radar_azimuth_batch: Array,
    geom: GeometryParams,
    device,
    active_dims=None,
    with_jacobian=False,
):
    """Project and rasterize a fixed 3D mesh for a batch of radar azimuths."""
    import torch

    validate_symmetric_geometry(geom)
    device = torch.device(device)
    points = map_points_xyz_torch(P_v, device)
    if not torch.is_tensor(triangles):
        triangle_array = np.asarray(triangles)
        if triangle_array.size == 0:
            raise ValueError("Mesh contains no triangles.")
        if np.min(triangle_array) < 0 or np.max(triangle_array) >= len(points):
            raise ValueError("Mesh triangle indices do not reference the supplied vertices.")
    faces = map_mesh_triangles_torch(triangles, device)

    active_dims = normalize_active_dims(active_dims) if with_jacobian else []
    T_batch_np = np.asarray([T @ odom_i for odom_i in odom_batch])
    T_batch = torch.as_tensor(T_batch_np, device=device, dtype=points.dtype)
    azimuths = torch.as_tensor(radar_azimuth_batch, device=device, dtype=points.dtype)

    if device.type == "cuda":
        torch.cuda.synchronize(device)
    t_total = perf_counter()

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

    batch_size = int(P_r_batch.shape[0])
    with torch.no_grad():
        t0 = perf_counter()
        candidate_mask = vertex_masks[:, faces].any(dim=2)
        candidate_batch_ids, candidate_face_ids = torch.nonzero(
            candidate_mask,
            as_tuple=True,
        )
        candidate_face_counts = torch.bincount(
            candidate_batch_ids,
            minlength=batch_size,
        )
        selected_vertex_counts = vertex_masks.sum(dim=1)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        face_selection_time = perf_counter() - t0

        t0 = perf_counter()
        if len(candidate_face_ids) == 0:
            I_faces = torch.empty((0, 3, 3), device=device, dtype=points.dtype)
            J_I_faces = None
            face_batch_ids = candidate_batch_ids
        else:
            candidate_vertex_ids = faces[candidate_face_ids]
            packed_vertex_keys = (
                candidate_batch_ids[:, None] * len(points)
                + candidate_vertex_ids
            )
            unique_vertex_keys, inverse = torch.unique(
                packed_vertex_keys.reshape(-1),
                sorted=False,
                return_inverse=True,
            )
            unique_batch_ids = torch.div(
                unique_vertex_keys,
                len(points),
                rounding_mode="floor",
            )
            unique_vertex_ids = torch.remainder(unique_vertex_keys, len(points))
            P_r_unique = P_r_batch[unique_batch_ids, unique_vertex_ids]
            I_unique, J_I_unique, x_local_unique = _project_packed_mesh_vertices(
                P_r=P_r_unique,
                azimuths=azimuths[unique_batch_ids],
                geom=geom,
                active_dims=active_dims,
                with_jacobian=with_jacobian,
            )
            inverse = inverse.reshape(-1, 3)
            front_facing_domain = (x_local_unique[inverse] > 0.0).all(dim=1)
            I_faces = I_unique[inverse][front_facing_domain]
            face_batch_ids = candidate_batch_ids[front_facing_domain]
            J_I_faces = None
            if J_I_unique is not None:
                J_I_faces = J_I_unique[inverse][front_facing_domain]
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        selected_geometry_time = perf_counter() - t0

        t0 = perf_counter()
        D_batch, J_D_batch, raster_stats = _rasterize_packed_mesh_faces_torch(
            I_faces=I_faces,
            J_I_faces=J_I_faces,
            face_batch_ids=face_batch_ids,
            batch_size=batch_size,
            geom=geom,
        )
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        rasterization_time = perf_counter() - t0

        t0 = perf_counter()
        D_batch_np = D_batch.detach().cpu().numpy()
        J_D_batch_np = None
        if J_D_batch is not None:
            J_D_batch_np = J_D_batch.detach().cpu().numpy()
        selected_vertex_counts_np = selected_vertex_counts.detach().cpu().numpy()
        candidate_face_counts_np = candidate_face_counts.detach().cpu().numpy()
        valid_face_counts_np = (
            raster_stats["valid_face_counts"].detach().cpu().numpy()
        )
        triangle_pixel_test_counts_np = (
            raster_stats["triangle_pixel_test_counts"].detach().cpu().numpy()
        )
        covered_pixel_counts_np = (
            raster_stats["covered_pixel_counts"].detach().cpu().numpy()
        )
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        geometry_to_cpu_time = perf_counter() - t0

    outputs = []
    for batch_idx in range(batch_size):
        J_np = None if J_D_batch_np is None else J_D_batch_np[batch_idx]
        outputs.append(
            (
                D_batch_np[batch_idx],
                J_np,
                {
                    "selected_vertices": int(selected_vertex_counts_np[batch_idx]),
                    "candidate_faces": int(candidate_face_counts_np[batch_idx]),
                    "valid_faces": int(valid_face_counts_np[batch_idx]),
                    "triangle_pixel_tests": int(
                        triangle_pixel_test_counts_np[batch_idx]
                    ),
                    "covered_pixels": int(covered_pixel_counts_np[batch_idx]),
                },
            )
        )

    timing_keys = (
        "point_transform_time_s",
        "cartesian_frustum_mask_time_s",
        "candidate_face_selection_time_s",
        "selected_geometry_time_s",
        "mesh_rasterization_time_s",
        "geometry_to_cpu_time_s",
    )
    timing_totals = {
        "point_transform_time_s": transform_time,
        "cartesian_frustum_mask_time_s": frustum_time,
        "candidate_face_selection_time_s": face_selection_time,
        "selected_geometry_time_s": selected_geometry_time,
        "mesh_rasterization_time_s": rasterization_time,
        "geometry_to_cpu_time_s": geometry_to_cpu_time,
    }
    timing_per_azimuth = {
        key: timing_totals[key] / max(batch_size, 1)
        for key in timing_keys
    }
    timing_per_azimuth["pose_to_pixel_call_time_s"] = sum(
        timing_per_azimuth[key] for key in timing_keys
    )
    timing_per_azimuth["single_azimuth_geometry_time_s"] = (
        timing_per_azimuth["pose_to_pixel_call_time_s"]
    )
    timing_per_azimuth["mesh_batch_wall_time_s"] = (
        perf_counter() - t_total
    ) / batch_size
    return outputs, timing_per_azimuth


def damping_matrix(H: Array, damping_mode: str = "identity") -> Array:
    H = np.asarray(H, dtype=float)

    if damping_mode == "identity":
        return np.eye(H.shape[0], dtype=H.dtype)
    elif damping_mode == "diag":
        return np.diag(np.maximum(np.diag(H), 1e-12))
    else:
        raise ValueError(f"Unknown damping_mode: {damping_mode}")


def solve_damped_gn_step(H: Array, g: Array, damping: float = 1e-3, damping_mode: str = "identity") -> Array:
    H = np.asarray(H, dtype=float)
    g = np.asarray(g, dtype=float).reshape(-1)
    D = damping_matrix(H, damping_mode)

    return -np.linalg.solve(H + damping * D, g)


def compute_cost_from_residual(e: Array) -> float:
    e = np.asarray(e, dtype=float).reshape(-1)
    return 0.5 * float(e @ e)


def stack_to_normal_equations(e_list: Sequence[Array], J_list: Sequence[Array]) -> LinearizationResult:
    if len(e_list) == 0:
        raise ValueError("No residual blocks were provided.")

    e = np.concatenate([np.asarray(ei).reshape(-1) for ei in e_list], axis=0)
    J = np.vstack([np.asarray(Ji) for Ji in J_list])
    H = J.T @ J
    g = J.T @ e
    return LinearizationResult(e=e, J=J, H=H, g=g, cost=compute_cost_from_residual(e))


def accumulate_normal_equations(e_list: Sequence[Array], J_list: Sequence[Array]) -> LinearizationResult:
    if len(e_list) == 0:
        raise ValueError("No residual blocks were provided.")

    dim = np.asarray(J_list[0]).shape[1]
    H = np.zeros((dim, dim), dtype=float)
    g = np.zeros(dim, dtype=float)
    cost = 0.0

    for e_i, J_i in zip(e_list, J_list):
        e_i = np.asarray(e_i, dtype=float).reshape(-1)
        J_i = np.asarray(J_i, dtype=float)
        H += J_i.T @ J_i
        g += J_i.T @ e_i
        cost += 0.5 * float(e_i @ e_i)

    return LinearizationResult(e=None, J=None, H=H, g=g, cost=cost)


def summarize_timing_diagnostics(diagnostics: Sequence[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    timing_rows = [row.get("timing", {}) for row in diagnostics if row.get("timing")]
    if not timing_rows:
        return None

    summary: Dict[str, float] = {"num_azimuths": float(len(timing_rows))}
    for key in sorted({key for timing in timing_rows for key in timing.keys()}):
        values = [float(timing.get(key, 0.0)) for timing in timing_rows]
        summary[f"{key}_sum"] = float(np.sum(values))
        summary[f"{key}_mean"] = float(np.mean(values))

    summary["normal_equation_time_s_sum"] = float(
        np.sum([float(row.get("normal_equation_time_s", 0.0)) for row in diagnostics])
    )
    return summary


def format_timing_summary(label: str, elapsed_time_s: float, diagnostics: Sequence[Dict[str, Any]]) -> Optional[str]:
    summary = summarize_timing_diagnostics(diagnostics)
    if summary is None:
        return None

    n = max(summary["num_azimuths"], 1.0)
    fields = [
        f"{label}: total {elapsed_time_s:.3f}s",
        f"az avg {elapsed_time_s / n:.3f}s",
        f"pose/pixel {summary.get('pose_to_pixel_call_time_s_sum', 0.0):.3f}s",
        f"transform {summary.get('point_transform_time_s_sum', 0.0):.3f}s",
        f"frustum {summary.get('cartesian_frustum_mask_time_s_sum', 0.0):.3f}s",
        f"face select {summary.get('candidate_face_selection_time_s_sum', 0.0):.3f}s",
        f"selected geom {summary.get('selected_geometry_time_s_sum', 0.0):.3f}s",
        f"mesh raster {summary.get('mesh_rasterization_time_s_sum', 0.0):.3f}s",
        f"geom->cpu {summary.get('geometry_to_cpu_time_s_sum', 0.0):.3f}s",
        f"point J {summary.get('se3_point_jacobian_only_time_s_sum', 0.0):.3f}s",
        f"cart->sph {summary.get('cartesian_to_spherical_time_s_sum', 0.0):.3f}s",
        f"sph J {summary.get('spherical_jacobian_only_time_s_sum', 0.0):.3f}s",
        f"sph->pixel {summary.get('spherical_to_pixel_coordinates_time_s_sum', 0.0):.3f}s",
        f"pixel J {summary.get('pixel_jacobian_scaling_time_s_sum', 0.0):.3f}s",
        f"delaunay {summary.get('delaunay_plan_time_s_sum', 0.0):.3f}s",
        f"depth/J {summary.get('depth_jacobian_time_s_sum', 0.0):.3f}s",
        f"model JVP {summary.get('model_jvp_time_s_sum', 0.0):.3f}s",
        f"cost forward {summary.get('cost_forward_time_s_sum', 0.0):.3f}s",
        f"model fwd {summary.get('model_forward_time_s_sum', 0.0):.3f}s",
        f"normal {summary.get('normal_equation_time_s_sum', 0.0):.3f}s",
    ]
    return "  timing " + " | ".join(fields)


def apply_model_output_activation_torch(y, output_activation="raw"):
    from localization.validate_jacobians_with_model_jvp import apply_model_output_activation
    return apply_model_output_activation(y, output_activation)


def model_jacobian_pose_jvp_active(
    model,
    D_np,
    J_D_eps_np,
    active_dims: Sequence[int],
    device=None,
    dtype=None,
    output_activation="raw",
):
    from localization.validate_jacobians_with_model_jvp import (
        freeze_model_parameters,
        infer_model_device_and_dtype,
        require_torch,
        torch_attention_jvp_compatibility,
        torch_jvp,
    )
    import torch

    require_torch()
    model.eval()
    freeze_model_parameters(model)
    device, dtype = infer_model_device_and_dtype(model, device=device, dtype=dtype)
    model = model.to(device=device, dtype=dtype)

    H, W = D_np.shape
    active_dims = list(active_dims)
    assert J_D_eps_np.shape == (H * W, len(active_dims))

    D = torch.as_tensor(D_np, dtype=dtype, device=device).view(1, 1, H, W)
    J_D_eps = torch.as_tensor(J_D_eps_np, dtype=dtype, device=device).view(H, W, len(active_dims))

    def f(D_in):
        return apply_model_output_activation_torch(model(D_in).reshape(-1), output_activation)

    with torch_attention_jvp_compatibility():
        m = f(D)
        R = m.numel()
        cols = []
        use_torch_func_jvp = torch_jvp is not None

        for local_col in range(len(active_dims)):
            dD = J_D_eps[:, :, local_col].view(1, 1, H, W)
            if use_torch_func_jvp:
                try:
                    _, dm = torch_jvp(f, (D,), (dD,))
                except NotImplementedError as err:
                    if "forward AD" not in str(err):
                        raise
                    use_torch_func_jvp = False
                    _, dm = torch.autograd.functional.jvp(f, D, dD, create_graph=False, strict=False)
            else:
                _, dm = torch.autograd.functional.jvp(f, D, dD, create_graph=False, strict=False)

            cols.append(dm.reshape(R))

    J = torch.stack(cols, dim=1)
    return m.detach().cpu().numpy(), J.detach().cpu().numpy()


def model_jacobian_pose_jvp_active_batch(
    model,
    D_batch_np,
    J_D_eps_batch_np,
    active_dims: Sequence[int],
    device=None,
    dtype=None,
    output_activation="raw",
):
    from localization.validate_jacobians_with_model_jvp import (
        freeze_model_parameters,
        infer_model_device_and_dtype,
        require_torch,
        torch_attention_jvp_compatibility,
        torch_jvp,
    )
    import torch

    require_torch()
    model.eval()
    freeze_model_parameters(model)
    device, dtype = infer_model_device_and_dtype(model, device=device, dtype=dtype)
    model = model.to(device=device, dtype=dtype)

    D_batch_np = np.asarray(D_batch_np)
    if D_batch_np.ndim != 3:
        raise ValueError(f"D_batch_np must have shape (B, H, W), got {D_batch_np.shape}.")

    B, H, W = D_batch_np.shape
    active_dims = list(active_dims)
    K = len(active_dims)

    J_D_eps_batch_np = np.asarray(J_D_eps_batch_np)
    if J_D_eps_batch_np.shape == (B, H * W, K):
        J_D_eps_batch_np = J_D_eps_batch_np.reshape(B, H, W, K)
    elif J_D_eps_batch_np.shape != (B, H, W, K):
        raise ValueError(
            f"J_D_eps_batch_np must have shape {(B, H * W, K)} or {(B, H, W, K)}, "
            f"got {J_D_eps_batch_np.shape}."
        )

    D = torch.as_tensor(D_batch_np, dtype=dtype, device=device).view(B, 1, H, W)
    J_D_eps = torch.as_tensor(J_D_eps_batch_np, dtype=dtype, device=device).view(B, H, W, K)

    def f(D_in):
        return apply_model_output_activation_torch(model(D_in), output_activation)

    with torch_attention_jvp_compatibility():
        m = f(D)
        if m.ndim != 2:
            m = m.reshape(B, -1)

        cols = []
        use_torch_func_jvp = torch_jvp is not None
        for local_col in range(K):
            dD = J_D_eps[:, :, :, local_col].view(B, 1, H, W)
            if use_torch_func_jvp:
                try:
                    _, dm = torch_jvp(f, (D,), (dD,))
                except NotImplementedError as err:
                    if "forward AD" not in str(err):
                        raise
                    use_torch_func_jvp = False
                    _, dm = torch.autograd.functional.jvp(f, D, dD, create_graph=False, strict=False)
            else:
                _, dm = torch.autograd.functional.jvp(f, D, dD, create_graph=False, strict=False)

            cols.append(dm.reshape(B, -1))

    J = torch.stack(cols, dim=2)
    return m.detach().cpu().numpy(), J.detach().cpu().numpy()


def compute_single_azimuth_geometry_active(
    P_v,
    T,
    odom_i,
    geom: GeometryParams,
    active_dims: Sequence[int],
    radar_azi=None,
):
    from localization.validate_jacobians_with_model_jvp import (
        barycentric_depths_and_J_I,
        build_barycentric_plan,
        d_pixel_deps_fast,
    )

    timing: Dict[str, float] = {}
    t_total = perf_counter()

    T_i = T @ odom_i

    t0 = perf_counter()
    J_I_eps_stacked, pix_cache = d_pixel_deps_fast(
        T_i,
        P_v,
        geom.theta_min,
        geom.theta_max,
        geom.phi_min,
        geom.phi_max,
        geom.dtheta,
        geom.dphi,
        radar_azi=radar_azi,
    )
    timing.update(pix_cache.get("timing", {}))
    timing["pose_to_pixel_call_time_s"] = perf_counter() - t0

    I = pix_cache["I"]
    J_I_eps_blocks = J_I_eps_stacked.reshape(-1, 3, 6)
    active_dims = list(active_dims)

    t0 = perf_counter()
    plan = build_barycentric_plan(
        I,
        width=geom.width,
        height=geom.height,
        max_uv_edge_length=geom.max_uv_edge_length,
        max_depth_jump=geom.max_depth_jump,
    )
    timing["delaunay_plan_time_s"] = perf_counter() - t0

    t0 = perf_counter()
    D, J_D_I, bary = barycentric_depths_and_J_I(I, plan, fill_value=geom.fill_value)
    J_I_active = J_I_eps_blocks[:, :, active_dims].reshape(-1, len(active_dims))
    J_D_active = np.asarray(J_D_I @ J_I_active)
    timing["depth_jacobian_time_s"] = perf_counter() - t0
    timing["single_azimuth_geometry_time_s"] = perf_counter() - t_total

    return D, J_D_active, {
        "I": I,
        "plan": plan,
        "D": D,
        "J_D_eps_active": J_D_active,
        "depth_cache": {"J_D_I": J_D_I, "bary": bary},
        "timing": timing,
    }


def finalize_selected_pixel_geometry(
    I,
    J_I_eps_blocks,
    geom: GeometryParams,
    active_dims: Sequence[int],
    base_timing: Optional[Dict[str, float]] = None,
):
    from localization.validate_jacobians_with_model_jvp import (
        barycentric_depths_and_J_I,
        build_barycentric_plan,
    )

    timing = dict(base_timing or {})
    t_total = perf_counter()

    t0 = perf_counter()
    plan = build_barycentric_plan(
        I,
        width=geom.width,
        height=geom.height,
        max_uv_edge_length=geom.max_uv_edge_length,
        max_depth_jump=geom.max_depth_jump,
    )
    timing["delaunay_plan_time_s"] = perf_counter() - t0

    t0 = perf_counter()
    D, J_D_I, bary = barycentric_depths_and_J_I(I, plan, fill_value=geom.fill_value)
    J_I_active = J_I_eps_blocks[:, :, list(active_dims)].reshape(-1, len(active_dims))
    J_D_active = np.asarray(J_D_I @ J_I_active)
    timing["depth_jacobian_time_s"] = perf_counter() - t0
    timing["single_azimuth_geometry_time_s"] = (
        timing.get("pose_to_pixel_call_time_s", 0.0)
        + timing["delaunay_plan_time_s"]
        + timing["depth_jacobian_time_s"]
    )

    return D, J_D_active, {
        "I": I,
        "plan": plan,
        "D": D,
        "J_D_eps_active": J_D_active,
        "depth_cache": {"J_D_I": J_D_I, "bary": bary},
        "timing": timing,
    }


def finalize_selected_pixel_depth(
    I,
    geom: GeometryParams,
):
    from localization.validate_jacobians_with_model_jvp import (
        barycentric_depths_from_plan,
        build_barycentric_plan,
    )

    plan = build_barycentric_plan(
        I,
        width=geom.width,
        height=geom.height,
        max_uv_edge_length=geom.max_uv_edge_length,
        max_depth_jump=geom.max_depth_jump,
    )
    D, _ = barycentric_depths_from_plan(I, plan, fill_value=geom.fill_value)
    return D


def compute_single_azimuth_model_jacobian_chain_active(
    P_v,
    T,
    odom_i,
    model,
    geom: GeometryParams,
    active_dims: Sequence[int],
    device=None,
    dtype=None,
    radar_azi=None,
    model_output_activation="raw",
):
    timing: Dict[str, float] = {}
    t_total = perf_counter()

    D, J_D_active, cache = compute_single_azimuth_geometry_active(
        P_v=P_v,
        T=T,
        odom_i=odom_i,
        geom=geom,
        active_dims=active_dims,
        radar_azi=radar_azi,
    )
    timing.update(cache.get("timing", {}))

    t0 = perf_counter()
    m, J_m_active = model_jacobian_pose_jvp_active(
        model,
        D,
        J_D_active,
        active_dims=active_dims,
        device=device,
        dtype=dtype,
        output_activation=model_output_activation,
    )
    timing["model_jvp_time_s"] = perf_counter() - t0
    timing["single_azimuth_total_time_s"] = perf_counter() - t_total
    cache["timing"] = timing

    return m, J_m_active, cache


def compute_cost_depth_patch(P_v, T_i, radar_azi, geom: GeometryParams):
    from localization.validate_jacobians_with_model_jvp import (
        barycentric_depths_from_plan,
        build_barycentric_plan,
        cartesian_to_spherical,
        keep_mask_from_spherical,
        select_spherical,
        spherical_to_pixel,
        wrap_to_pi,
    )

    X = T_i @ P_v
    P_r = X[:, :3, :]
    S = cartesian_to_spherical(P_r)
    S[:, 1, 0] = wrap_to_pi(S[:, 1, 0] - radar_azi)
    mask = keep_mask_from_spherical(S, geom.theta_min, geom.theta_max, geom.phi_min, geom.phi_max)
    S_kept = select_spherical(S, mask)
    I = spherical_to_pixel(S_kept, theta_min=geom.theta_min, dtheta=geom.dtheta, phi_max=geom.phi_max, dphi=geom.dphi)
    plan = build_barycentric_plan(
        I,
        width=geom.width,
        height=geom.height,
        max_uv_edge_length=geom.max_uv_edge_length,
        max_depth_jump=geom.max_depth_jump,
    )
    D, _ = barycentric_depths_from_plan(I, plan, fill_value=geom.fill_value)
    return D


def model_forward_batch_numpy(model, D_batch_np, device=None, dtype=None, output_activation="raw"):
    from localization.validate_jacobians_with_model_jvp import (
        freeze_model_parameters,
        infer_model_device_and_dtype,
        require_torch,
        torch_attention_jvp_compatibility,
    )
    import torch

    require_torch()
    model.eval()
    freeze_model_parameters(model)
    device, dtype = infer_model_device_and_dtype(model, device=device, dtype=dtype)
    model = model.to(device=device, dtype=dtype)

    D = torch.as_tensor(D_batch_np, dtype=dtype, device=device).unsqueeze(1)
    with torch.no_grad():
        with torch_attention_jvp_compatibility():
            m = apply_model_output_activation_torch(model(D), output_activation)
    return m.detach().cpu().numpy()


def selected_azimuth_indices(odom_transforms: Array, options: ResidualBuildOptions) -> List[int]:
    if options.azimuth_indices is None:
        return list(range(len(odom_transforms)))
    return [int(i) for i in options.azimuth_indices]


def compute_radar_residual_and_jacobian_fast(
    P_v: Array,
    T: Array,
    odom_transforms: Array,
    m_obs_all: Array,
    model: Any,
    geom: GeometryParams,
    options: Optional[ResidualBuildOptions] = None,
    radar_azimuths: Optional[Array] = None,
    mesh_triangles: Optional[Array] = None,
) -> LinearizationResult:
    if options is None:
        options = ResidualBuildOptions()

    active_dims = normalize_active_dims(options.active_dims)
    azimuth_indices = selected_azimuth_indices(odom_transforms, options)
    radar_azimuths = None if radar_azimuths is None else np.asarray(radar_azimuths).reshape(-1)

    e_list: List[Array] = []
    J_list: List[Array] = []
    diagnostics: List[Dict[str, Any]] = []
    batch_size = max(int(options.linearization_batch_size), 1)

    pending_D: List[Array] = []
    pending_J_D: List[Array] = []
    pending_obs: List[Array] = []
    pending_azimuths: List[int] = []
    pending_caches: List[Dict[str, Any]] = []

    def flush_linearization_batch():
        if not pending_D:
            return

        t_model = perf_counter()
        m_batch, J_m_batch = model_jacobian_pose_jvp_active_batch(
            model,
            np.stack(pending_D).astype(np.float32, copy=False),
            np.stack(pending_J_D).astype(np.float32, copy=False),
            active_dims=active_dims,
            device=options.device,
            dtype=options.dtype,
            output_activation=options.model_output_activation,
        )
        model_jvp_time_s = perf_counter() - t_model
        per_row_model_time_s = model_jvp_time_s / max(len(pending_D), 1)

        for local_idx, (i, m_pred_i, J_m_active_i, m_obs_i, cache_i) in enumerate(
            zip(pending_azimuths, m_batch, J_m_batch, pending_obs, pending_caches)
        ):
            m_pred_i = np.asarray(m_pred_i).reshape(-1)
            m_obs_i = np.asarray(m_obs_i).reshape(-1)
            if m_obs_i.shape != m_pred_i.shape:
                raise ValueError(f"m_obs_all[{i}] has shape {m_obs_i.shape}, but model predicted {m_pred_i.shape}.")

            e_i = m_obs_i - m_pred_i
            J_e_i = np.zeros((e_i.size, 6), dtype=float)
            J_e_i[:, active_dims] = -np.asarray(J_m_active_i)

            e_list.append(e_i)
            J_list.append(J_e_i)

            if options.keep_diagnostics:
                timing = dict(cache_i.get("timing", {}))
                timing["model_jvp_time_s"] = per_row_model_time_s
                timing["batched_model_jvp_time_s"] = model_jvp_time_s
                timing["linearization_batch_size"] = float(len(pending_D))
                timing["single_azimuth_total_time_s"] = (
                    timing.get("single_azimuth_geometry_time_s", 0.0)
                    + per_row_model_time_s
                )
                diagnostics.append({
                    "azimuth": i,
                    "cost": 0.5 * float(e_i @ e_i),
                    "residual_norm": float(np.linalg.norm(e_i)),
                    "m_pred_shape": m_pred_i.shape,
                    "J_shape": J_e_i.shape,
                    "active_dims": active_dims,
                    "num_selected_points": int(cache_i["I"].shape[0]) if "I" in cache_i else None,
                    "num_valid_pixels": int(cache_i["plan"]["pixel_ids"].shape[0]) if "plan" in cache_i else None,
                    "num_candidate_faces": cache_i.get("candidate_faces"),
                    "num_covered_pixels": cache_i.get("covered_pixels"),
                    "timing": timing,
                })

        pending_D.clear()
        pending_J_D.clear()
        pending_obs.clear()
        pending_azimuths.clear()
        pending_caches.clear()

    def queue_geometry(i, D_i, J_D_active_i, cache_i):
        geometry_time_s = float(cache_i.get("timing", {}).get("single_azimuth_geometry_time_s", 0.0))
        if options.keep_diagnostics:
            cache_i.setdefault("timing", {})
            cache_i["timing"]["chain_call_time_s"] = geometry_time_s

        pending_D.append(D_i)
        pending_J_D.append(J_D_active_i)
        pending_obs.append(np.asarray(m_obs_all[i]).reshape(-1))
        pending_azimuths.append(i)
        pending_caches.append(cache_i)

        if len(pending_D) >= batch_size:
            flush_linearization_batch()

    if options.geometry_backend == "torch_frustum":
        geometry_batch_size = max(int(options.geometry_batch_size), 1)
        radar_azimuth_values = (
            np.zeros(len(odom_transforms), dtype=float)
            if radar_azimuths is None
            else radar_azimuths
        )
        for start in range(0, len(azimuth_indices), geometry_batch_size):
            batch_indices = azimuth_indices[start:start + geometry_batch_size]
            pixel_outputs, batch_timing = torch_frustum_pixel_geometry_batch(
                P_v=P_v,
                T=T,
                odom_batch=odom_transforms[batch_indices],
                radar_azimuth_batch=radar_azimuth_values[batch_indices],
                geom=geom,
                device=options.device,
                with_jacobian=True,
            )
            for i, (I_i, J_I_i, _) in zip(batch_indices, pixel_outputs):
                D_i, J_D_active_i, cache_i = finalize_selected_pixel_geometry(
                    I=I_i,
                    J_I_eps_blocks=J_I_i,
                    geom=geom,
                    active_dims=active_dims,
                    base_timing=batch_timing,
                )
                queue_geometry(i, D_i, J_D_active_i, cache_i)
    elif options.geometry_backend == "torch_mesh":
        if mesh_triangles is None:
            raise ValueError("mesh_triangles is required for geometry_backend='torch_mesh'.")
        geometry_batch_size = max(int(options.geometry_batch_size), 1)
        radar_azimuth_values = (
            np.zeros(len(odom_transforms), dtype=float)
            if radar_azimuths is None
            else radar_azimuths
        )
        for start in range(0, len(azimuth_indices), geometry_batch_size):
            batch_indices = azimuth_indices[start:start + geometry_batch_size]
            mesh_outputs, batch_timing = torch_mesh_depth_geometry_batch(
                P_v=P_v,
                triangles=mesh_triangles,
                T=T,
                odom_batch=odom_transforms[batch_indices],
                radar_azimuth_batch=radar_azimuth_values[batch_indices],
                geom=geom,
                device=options.device,
                active_dims=active_dims,
                with_jacobian=True,
            )
            for i, (D_i, J_D_active_i, mesh_stats) in zip(batch_indices, mesh_outputs):
                cache_i = {
                    "D": D_i,
                    "J_D_eps_active": J_D_active_i,
                    "timing": dict(batch_timing),
                    **mesh_stats,
                }
                queue_geometry(i, D_i, J_D_active_i, cache_i)
    elif options.geometry_backend == "numpy_spherical":
        for i in azimuth_indices:
            t0 = perf_counter()
            D_i, J_D_active_i, cache_i = compute_single_azimuth_geometry_active(
                P_v=P_v,
                T=T,
                odom_i=odom_transforms[i],
                geom=geom,
                active_dims=active_dims,
                radar_azi=None if radar_azimuths is None else radar_azimuths[i],
            )
            cache_i.setdefault("timing", {})
            cache_i["timing"]["single_azimuth_geometry_time_s"] = perf_counter() - t0
            queue_geometry(i, D_i, J_D_active_i, cache_i)
    else:
        raise ValueError(f"Unknown geometry_backend: {options.geometry_backend}")

    flush_linearization_batch()

    t_normal = perf_counter()
    if options.mode == "stack":
        lin = stack_to_normal_equations(e_list, J_list)
    elif options.mode == "accumulate":
        lin = accumulate_normal_equations(e_list, J_list)
    else:
        raise ValueError(f"Unknown mode: {options.mode}")
    normal_time_s = perf_counter() - t_normal

    lin.diagnostics = diagnostics
    if options.keep_diagnostics:
        lin.diagnostics.append({"azimuth": None, "normal_equation_time_s": normal_time_s})
    return lin


def compute_radar_cost_only_fast(
    P_v: Array,
    T: Array,
    odom_transforms: Array,
    m_obs_all: Array,
    model: Any,
    geom: GeometryParams,
    options: Optional[ResidualBuildOptions] = None,
    radar_azimuths: Optional[Array] = None,
    mesh_triangles: Optional[Array] = None,
) -> CostResult:
    if options is None:
        options = ResidualBuildOptions()

    azimuth_indices = selected_azimuth_indices(odom_transforms, options)
    radar_azimuths = None if radar_azimuths is None else np.asarray(radar_azimuths).reshape(-1)
    batch_size = max(int(options.candidate_batch_size), 1)

    cost = 0.0
    diagnostics: List[Dict[str, Any]] = []
    patches = []
    obs_rows = []
    az_rows = []
    forward_times = []
    geometry_timings = []

    def flush_batch():
        nonlocal cost
        if not patches:
            return

        t_model = perf_counter()
        preds = model_forward_batch_numpy(
            model,
            np.stack(patches).astype(np.float32, copy=False),
            device=options.device,
            dtype=options.dtype,
            output_activation=options.model_output_activation,
        )
        model_time_s = perf_counter() - t_model

        for local_idx, pred in enumerate(preds):
            obs = obs_rows[local_idx]
            e = np.asarray(obs).reshape(-1) - np.asarray(pred).reshape(-1)
            cost += 0.5 * float(e @ e)
            if options.keep_diagnostics:
                timing = dict(geometry_timings[local_idx])
                timing["cost_forward_time_s"] = forward_times[local_idx]
                timing["model_forward_time_s"] = model_time_s / max(len(preds), 1)
                diagnostics.append({
                    "azimuth": az_rows[local_idx],
                    "cost": 0.5 * float(e @ e),
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
        if len(patches) >= batch_size:
            flush_batch()

    if options.geometry_backend == "torch_frustum":
        geometry_batch_size = max(int(options.geometry_batch_size), 1)
        radar_azimuth_values = (
            np.zeros(len(odom_transforms), dtype=float)
            if radar_azimuths is None
            else radar_azimuths
        )
        for start in range(0, len(azimuth_indices), geometry_batch_size):
            batch_indices = azimuth_indices[start:start + geometry_batch_size]
            t_forward = perf_counter()
            pixel_outputs, batch_timing = torch_frustum_pixel_geometry_batch(
                P_v=P_v,
                T=T,
                odom_batch=odom_transforms[batch_indices],
                radar_azimuth_batch=radar_azimuth_values[batch_indices],
                geom=geom,
                device=options.device,
                with_jacobian=False,
            )
            pixel_time_s = perf_counter() - t_forward
            for i, (I_i, _, _) in zip(batch_indices, pixel_outputs):
                t_depth = perf_counter()
                D = finalize_selected_pixel_depth(I_i, geom)
                depth_time_s = perf_counter() - t_depth
                timing = dict(batch_timing)
                timing["delaunay_depth_time_s"] = depth_time_s
                queue_depth(i, D, pixel_time_s / max(len(batch_indices), 1) + depth_time_s, timing)
    elif options.geometry_backend == "torch_mesh":
        if mesh_triangles is None:
            raise ValueError("mesh_triangles is required for geometry_backend='torch_mesh'.")
        geometry_batch_size = max(int(options.geometry_batch_size), 1)
        radar_azimuth_values = (
            np.zeros(len(odom_transforms), dtype=float)
            if radar_azimuths is None
            else radar_azimuths
        )
        for start in range(0, len(azimuth_indices), geometry_batch_size):
            batch_indices = azimuth_indices[start:start + geometry_batch_size]
            t_forward = perf_counter()
            mesh_outputs, batch_timing = torch_mesh_depth_geometry_batch(
                P_v=P_v,
                triangles=mesh_triangles,
                T=T,
                odom_batch=odom_transforms[batch_indices],
                radar_azimuth_batch=radar_azimuth_values[batch_indices],
                geom=geom,
                device=options.device,
                with_jacobian=False,
            )
            batch_wall_time_s = perf_counter() - t_forward
            for i, (D, _, mesh_stats) in zip(batch_indices, mesh_outputs):
                timing = dict(batch_timing)
                timing.update(
                    {
                        "candidate_faces": float(mesh_stats["candidate_faces"]),
                        "covered_pixels": float(mesh_stats["covered_pixels"]),
                    }
                )
                queue_depth(
                    i,
                    D,
                    batch_wall_time_s / max(len(batch_indices), 1),
                    timing,
                )
    elif options.geometry_backend == "numpy_spherical":
        for i in azimuth_indices:
            t_forward = perf_counter()
            T_i = T @ odom_transforms[i]
            D = compute_cost_depth_patch(
                P_v=P_v,
                T_i=T_i,
                radar_azi=0.0 if radar_azimuths is None else radar_azimuths[i],
                geom=geom,
            )
            queue_depth(i, D, perf_counter() - t_forward, {})
    else:
        raise ValueError(f"Unknown geometry_backend: {options.geometry_backend}")

    flush_batch()
    return CostResult(cost=cost, diagnostics=diagnostics)


def left_se3_retract(T: Array, delta: Array) -> Array:
    from pylgmath.se3 import operations as se3op
    return se3op.vec2tran(np.asarray(delta).reshape(6, 1)) @ T


def levenberg_marquardt_optimize_fast(
    initial_state: Any,
    residual_jacobian_fn: Callable[[Any], LinearizationResult],
    cost_fn: Callable[[Any], CostResult],
    retract_fn: Callable[[Any, Array], Any],
    active_dims: Optional[Sequence[int]] = None,
    max_iters: int = 10,
    initial_damping: float = 1e-3,
    damping_mode: str = "identity",
    step_tol: float = 1e-8,
    grad_tol: float = 1e-8,
    cost_tol: float = 1e-12,
    accept_decrease_only: bool = True,
    damping_decrease: float = 1.0,
    damping_increase: float = 1.0,
    min_damping: float = 1e-12,
    max_damping: float = 1e12,
    use_alpha_line_search: bool = False,
    initial_alpha: float | Sequence[float] = 1.0,
    alpha_shrink: float = 0.5,
    max_alpha_attempts: int = 1,
    use_momentum: bool = False,
    momentum_beta: float = 0.8,
    max_cost_increase_ratio: float = 1e-3,
    max_iters_without_best_improvement: int = 3,
    verbose: bool = True,
) -> GNResult:
    initial_alpha = np.asarray(initial_alpha, dtype=float)
    if initial_alpha.ndim == 0:
        initial_alpha = np.full(6, float(initial_alpha))
    if initial_alpha.shape != (6,) or np.any(initial_alpha <= 0.0):
        raise ValueError("initial_alpha must be a positive scalar or length-6 sequence.")
    if not 0.0 < alpha_shrink <= 1.0:
        raise ValueError("alpha_shrink must be in (0, 1).")
    if max_alpha_attempts < 1:
        raise ValueError("max_alpha_attempts must be at least 1.")
    if use_momentum and use_alpha_line_search:
        raise ValueError("Momentum and alpha line search cannot be enabled together.")
    if not 0.0 <= momentum_beta < 1.0:
        raise ValueError("momentum_beta must be in [0, 1).")
    if max_cost_increase_ratio < 0.0:
        raise ValueError("max_cost_increase_ratio must be nonnegative.")
    if max_iters_without_best_improvement < 1:
        raise ValueError("max_iters_without_best_improvement must be at least 1.")

    state = initial_state
    damping = float(initial_damping)
    current_alpha = initial_alpha.copy()
    velocity = None
    best_state = copy.deepcopy(initial_state)
    best_cost = float("inf")
    iters_without_best_improvement = 0
    alpha_attempts = 0
    num_alpha_attempts = max_alpha_attempts if use_alpha_line_search else 5
    history: List[Dict[str, Any]] = []
    solve_dims = normalize_active_dims(active_dims)

    for it in range(max_iters):
        # if alpha_attempts >= num_alpha_attempts:
        #     break

        t0 = perf_counter()
        lin = residual_jacobian_fn(state)
        t_linearize = perf_counter() - t0

        H = np.asarray(lin.H, dtype=float)
        g = np.asarray(lin.g, dtype=float).reshape(-1)
        H_solve = H[np.ix_(solve_dims, solve_dims)]
        g_solve = g[solve_dims]
        cost = float(lin.cost)
        grad_norm = float(np.linalg.norm(g_solve))
        if cost < best_cost:
            best_cost = cost
            best_state = copy.deepcopy(state)
            iters_without_best_improvement = 0

        try:
            D_solve = damping_matrix(H_solve, damping_mode)
            cond_H = float(np.linalg.cond(H_solve))
            cond_H_damped = float(np.linalg.cond(H_solve + damping * D_solve))
        except np.linalg.LinAlgError:
            cond_H = float("inf")
            cond_H_damped = float("inf")

        delta_solve = solve_damped_gn_step(H_solve, g_solve, damping=damping, damping_mode=damping_mode)
        delta = np.zeros_like(g)
        delta[solve_dims] = delta_solve
        delta_norm_raw = float(np.linalg.norm(delta))

        if use_momentum:
            if velocity is None:
                velocity = delta.copy()
            else:
                velocity = momentum_beta * velocity + (1.0 - momentum_beta) * delta
            step_direction = velocity
            alpha = current_alpha.copy()
        else:
            step_direction = delta
            alpha = initial_alpha.copy() if use_alpha_line_search else np.ones(6)

        candidate = None
        cand_cost_result = None
        cand_cost = float("inf")
        t1 = perf_counter()

        delta_trial = alpha * step_direction
        candidate = retract_fn(state, delta_trial)
        cand_cost_result = cost_fn(candidate)
        cand_cost = float(cand_cost_result.cost)
        
        # while alpha_attempts < num_alpha_attempts:
        #     delta_trial = alpha * step_direction
        #     candidate = retract_fn(state, delta_trial)
        #     cand_cost_result = cost_fn(candidate)
        #     cand_cost = float(cand_cost_result.cost)

        #     # break

        #     if not accept_decrease_only or cand_cost < cost:
        #         break
        #     else:
        #         alpha *= alpha_shrink
        #         alpha_attempts += 1

        t_candidate = perf_counter() - t1
        delta_applied = alpha * step_direction
        delta_norm_applied = float(np.linalg.norm(delta_applied))

        allowed_cost_increase = (
            max_cost_increase_ratio * max(abs(cost), 1e-30)
            if use_momentum
            else 0.0
        )
        accepted_uphill = use_momentum and cost < cand_cost <= cost + allowed_cost_increase
        if use_momentum:
            accepted = cand_cost <= cost + allowed_cost_increase
        else:
            accepted = (cand_cost < cost) if accept_decrease_only else True

        best_improved = accepted and cand_cost < best_cost
        if accepted:
            state = candidate
            damping = max(min_damping, damping * damping_decrease)
            if best_improved:
                best_cost = cand_cost
                best_state = copy.deepcopy(candidate)
                iters_without_best_improvement = 0
            else:
                iters_without_best_improvement += 1
        else:
            damping = min(max_damping, damping * damping_increase)
            if use_momentum:
                velocity = None
                current_alpha *= alpha_shrink
                alpha_attempts += 1
            iters_without_best_improvement += 1

        actual_decrease = cost - cand_cost
        row = {
            "iter": it,
            "cost": cost,
            "candidate_cost": cand_cost,
            "actual_decrease": actual_decrease,
            "relative_decrease": actual_decrease / max(cost, 1e-30),
            "accepted": accepted,
            "accepted_uphill": accepted_uphill,
            "best_improved": best_improved,
            "best_cost": best_cost,
            "damping": damping,
            "delta": delta.copy(),
            "delta_applied": delta_applied.copy(),
            "alpha": alpha.copy(),
            "next_alpha": current_alpha.copy(),
            "alpha_attempts": alpha_attempts,
            "momentum_enabled": use_momentum,
            "velocity": None if velocity is None else velocity.copy(),
            "allowed_cost_increase": allowed_cost_increase,
            "iters_without_best_improvement": iters_without_best_improvement,
            "momentum_reset": False,
            "delta_norm": delta_norm_applied,
            "delta_norm_raw": delta_norm_raw,
            "delta_norm_applied": delta_norm_applied,
            "translation_step_norm": float(np.linalg.norm(delta_applied[:3])),
            "rotation_step_norm": float(np.linalg.norm(delta_applied[3:])),
            "grad_norm": grad_norm,
            "cond_H": cond_H,
            "cond_H_damped": cond_H_damped,
            "linearize_time_s": t_linearize,
            "candidate_eval_time_s": t_candidate,
            "diagnostics": lin.diagnostics,
            "candidate_diagnostics": cand_cost_result.diagnostics,
        }
        history.append(row)

        if verbose:
            if accepted_uphill:
                status = "ACCEPT_UPHILL"
            else:
                status = "ACCEPT" if accepted else "REJECT"
            delta_print = delta_applied.copy()
            delta_print[3:] = np.rad2deg(delta_print[3:])
            print(
                f"iter {it:02d} | {status} | "
                f"cost {cost:.6e} -> {cand_cost:.6e} | "
                # f"|delta| raw {delta_norm_raw:.3e} applied {delta_norm_applied:.3e} | "
                f"|delta applied| {np.array2string(delta_print, precision=4, separator=', ')} | "
                f"alpha {np.array2string(alpha, precision=3)} ({alpha_attempts} tries) | "
                f"|g| {grad_norm:.3e} | cond(H) {cond_H:.3e} | cond(Hd) {cond_H_damped:.3e}"
                # f"lambda {damping:.3e}"
            )
            current_timing = format_timing_summary("current", t_linearize, lin.diagnostics)
            candidate_timing = format_timing_summary("candidate cost-only", t_candidate, cand_cost_result.diagnostics)
            if current_timing is not None:
                print(current_timing)
            if candidate_timing is not None:
                print(candidate_timing)

        if accepted:
            if delta_norm_applied < step_tol:
                if verbose:
                    print("Converged: step norm below tolerance.")
                break
            if grad_norm < grad_tol:
                if verbose:
                    print("Converged: gradient norm below tolerance.")
                break
            if actual_decrease >= 0.0 and abs(actual_decrease) < cost_tol:
                if verbose:
                    print("Converged: cost decrease below tolerance.")
                break
        elif use_alpha_line_search and alpha_attempts >= max_alpha_attempts:
            if verbose:
                print("Stopped: alpha line search found no decreasing step.")
            break
        if use_momentum and iters_without_best_improvement >= max_iters_without_best_improvement:
            velocity = None
            # current_alpha = float(initial_alpha)
            current_alpha *= alpha_shrink
            alpha_attempts += 1
            iters_without_best_improvement = 0
            history[-1]["momentum_reset"] = True
            history[-1]["next_alpha"] = current_alpha
            if verbose:
                print("Reset momentum: no best-cost improvement within patience limit.")

    return GNResult(state=best_state if use_momentum else state, history=history)


def run_radar_lidar_localization_gn_fast(
    P_v: Array,
    T_init: Array,
    odom_transforms: Array,
    m_obs_all: Array,
    model: Any,
    geom: GeometryParams,
    options: Optional[ResidualBuildOptions] = None,
    radar_azimuths: Optional[Array] = None,
    mesh_triangles: Optional[Array] = None,
    max_iters: int = 10,
    initial_damping: float = 1e-3,
    damping_mode: str = "identity",
    active_dims: Optional[Sequence[int]] = None,
    use_alpha_line_search: bool = False,
    initial_alpha: float | Sequence[float] = 1.0,
    alpha_shrink: float = 0.5,
    max_alpha_attempts: int = 1,
    use_momentum: bool = False,
    momentum_beta: float = 0.8,
    max_cost_increase_ratio: float = 1e-3,
    max_iters_without_best_improvement: int = 3,
    verbose: bool = True,
) -> GNResult:
    if options is None:
        options = ResidualBuildOptions()
    options.active_dims = active_dims

    def residual_jacobian_fn(T_current: Array) -> LinearizationResult:
        return compute_radar_residual_and_jacobian_fast(
            P_v=P_v,
            mesh_triangles=mesh_triangles,
            T=T_current,
            odom_transforms=odom_transforms,
            m_obs_all=m_obs_all,
            model=model,
            geom=geom,
            options=options,
            radar_azimuths=radar_azimuths,
        )

    def cost_fn(T_current: Array) -> CostResult:
        return compute_radar_cost_only_fast(
            P_v=P_v,
            mesh_triangles=mesh_triangles,
            T=T_current,
            odom_transforms=odom_transforms,
            m_obs_all=m_obs_all,
            model=model,
            geom=geom,
            options=options,
            radar_azimuths=radar_azimuths,
        )

    return levenberg_marquardt_optimize_fast(
        initial_state=T_init,
        residual_jacobian_fn=residual_jacobian_fn,
        cost_fn=cost_fn,
        retract_fn=left_se3_retract,
        active_dims=active_dims,
        max_iters=max_iters,
        initial_damping=initial_damping,
        damping_mode=damping_mode,
        use_alpha_line_search=use_alpha_line_search,
        initial_alpha=initial_alpha,
        alpha_shrink=alpha_shrink,
        max_alpha_attempts=max_alpha_attempts,
        use_momentum=use_momentum,
        momentum_beta=momentum_beta,
        max_cost_increase_ratio=max_cost_increase_ratio,
        max_iters_without_best_improvement=max_iters_without_best_improvement,
        verbose=verbose,
    )
