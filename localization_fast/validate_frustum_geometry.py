#!/usr/bin/env python3
from __future__ import annotations

import argparse
from time import perf_counter

import numpy as np

from localization.validate_jacobians_with_model_jvp import (
    barycentric_depths_and_J_I,
    build_barycentric_plan,
    d_pixel_deps_fast,
)
from localization_fast.gauss_newton_localization_fast import (
    GeometryParams,
    torch_frustum_pixel_geometry_batch,
)


def make_transform(rng):
    angles = rng.normal(scale=0.08, size=3)
    cx, cy, cz = np.cos(angles)
    sx, sy, sz = np.sin(angles)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = rng.normal(scale=0.3, size=3)
    return T


def main():
    parser = argparse.ArgumentParser(
        description="Validate Torch Cartesian-frustum geometry against the NumPy spherical reference."
    )
    parser.add_argument("--num-points", type=int, default=200_000)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    if args.warmup < 0:
        raise ValueError("--warmup must be nonnegative.")
    if args.repeats < 1:
        raise ValueError("--repeats must be at least 1.")

    rng = np.random.default_rng(args.seed)
    points = rng.normal(size=(args.num_points, 3))
    points[:, 0] = rng.uniform(1.0, 80.0, size=args.num_points)
    points[:, 1] *= 18.0
    points[:, 2] *= 3.0

    P_v = np.ones((args.num_points, 4, 1))
    P_v[:, :3, 0] = points

    geom = GeometryParams(
        theta_min=np.deg2rad(-3.0),
        theta_max=np.deg2rad(3.0),
        phi_min=np.deg2rad(-3.0),
        phi_max=np.deg2rad(3.0),
        dtheta=np.deg2rad(0.1),
        dphi=np.deg2rad(0.1),
        width=60,
        height=60,
        max_depth_jump=2.0,
    )

    T = make_transform(rng)
    odom = np.stack([make_transform(rng) for _ in range(args.batch_size)])
    azimuths = rng.uniform(-np.pi, np.pi, size=args.batch_size)

    for _ in range(args.warmup):
        torch_frustum_pixel_geometry_batch(
            P_v=P_v,
            T=T,
            odom_batch=odom,
            radar_azimuth_batch=azimuths,
            geom=geom,
            device=args.device,
            with_jacobian=True,
        )

    torch_elapsed_values = []
    torch_timing_values = []
    torch_outputs = None
    for _ in range(args.repeats):
        t0 = perf_counter()
        torch_outputs, torch_timing = torch_frustum_pixel_geometry_batch(
            P_v=P_v,
            T=T,
            odom_batch=odom,
            radar_azimuth_batch=azimuths,
            geom=geom,
            device=args.device,
            with_jacobian=True,
        )
        torch_elapsed_values.append(perf_counter() - t0)
        torch_timing_values.append(torch_timing)

    torch_elapsed = float(np.mean(torch_elapsed_values))
    torch_timing = {
        key: float(np.mean([timing[key] for timing in torch_timing_values]))
        for key in torch_timing_values[0]
    }

    selected_mismatches = 0
    max_pixel_error = 0.0
    max_jacobian_error = 0.0
    max_depth_error = 0.0
    max_depth_jacobian_error = 0.0
    pixel_jacobian_diff_sq = 0.0
    pixel_jacobian_ref_sq = 0.0
    depth_jacobian_diff_sq = 0.0
    depth_jacobian_ref_sq = 0.0

    t0 = perf_counter()
    for batch_idx, (I_torch, J_torch, selected_count) in enumerate(torch_outputs):
        J_ref, cache_ref = d_pixel_deps_fast(
            T @ odom[batch_idx],
            P_v,
            geom.theta_min,
            geom.theta_max,
            geom.phi_min,
            geom.phi_max,
            geom.dtheta,
            geom.dphi,
            radar_azi=azimuths[batch_idx],
        )
        I_ref = cache_ref["I"]
        J_ref = J_ref.reshape(-1, 3, 6)

        if selected_count != I_ref.shape[0] or I_torch.shape != I_ref.shape:
            selected_mismatches += 1
            continue

        if I_ref.size:
            max_pixel_error = max(max_pixel_error, float(np.max(np.abs(I_torch - I_ref))))
            max_jacobian_error = max(max_jacobian_error, float(np.max(np.abs(J_torch - J_ref))))
            pixel_jacobian_diff_sq += float(np.sum((J_torch - J_ref) ** 2))
            pixel_jacobian_ref_sq += float(np.sum(J_ref ** 2))

        plan_ref = build_barycentric_plan(
            I_ref,
            width=geom.width,
            height=geom.height,
            max_uv_edge_length=geom.max_uv_edge_length,
            max_depth_jump=geom.max_depth_jump,
        )
        D_ref, J_D_I_ref, _ = barycentric_depths_and_J_I(I_ref, plan_ref, fill_value=geom.fill_value)
        D_torch, J_D_I_torch, _ = barycentric_depths_and_J_I(
            I_torch,
            plan_ref,
            fill_value=geom.fill_value,
        )
        J_D_ref = np.asarray(J_D_I_ref @ J_ref.reshape(-1, 6))
        J_D_torch = np.asarray(J_D_I_torch @ J_torch.reshape(-1, 6))
        max_depth_error = max(max_depth_error, float(np.max(np.abs(D_torch - D_ref))))
        max_depth_jacobian_error = max(
            max_depth_jacobian_error,
            float(np.max(np.abs(J_D_torch - J_D_ref))),
        )
        depth_jacobian_diff_sq += float(np.sum((J_D_torch - J_D_ref) ** 2))
        depth_jacobian_ref_sq += float(np.sum(J_D_ref ** 2))
    numpy_elapsed = perf_counter() - t0

    print("Frustum geometry validation")
    print(f"device:                       {args.device}")
    print(f"points:                       {args.num_points}")
    print(f"azimuth batch:                {args.batch_size}")
    print(f"warmup runs:                  {args.warmup}")
    print(f"timed runs:                   {args.repeats}")
    print(f"selected-count mismatches:    {selected_mismatches}")
    print(f"max pixel abs error:          {max_pixel_error:.9e}")
    print(f"max pixel Jacobian abs error: {max_jacobian_error:.9e}")
    print(
        "pixel Jacobian relative L2:  "
        f"{np.sqrt(pixel_jacobian_diff_sq / max(pixel_jacobian_ref_sq, 1e-30)):.9e}"
    )
    print(f"max depth abs error:          {max_depth_error:.9e}")
    print(f"max depth Jacobian abs error: {max_depth_jacobian_error:.9e}")
    print(
        "depth Jacobian relative L2:  "
        f"{np.sqrt(depth_jacobian_diff_sq / max(depth_jacobian_ref_sq, 1e-30)):.9e}"
    )
    print(f"Torch batch elapsed mean:     {torch_elapsed:.6f}s")
    print(f"Torch batch elapsed min:      {min(torch_elapsed_values):.6f}s")
    print(f"NumPy reference elapsed:      {numpy_elapsed:.6f}s")
    print(f"Torch timing per azimuth:     {torch_timing}")

    if selected_mismatches:
        raise SystemExit("Validation failed: selected point counts differ.")


if __name__ == "__main__":
    main()
