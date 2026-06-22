#!/usr/bin/env python3
"""Focused checks for the fixed-connectivity mesh depth rasterizer."""

import numpy as np
import torch
from pylgmath.se3 import operations as se3op

from localization_fast.gauss_newton_localization_fast import (
    GeometryParams,
    _rasterize_projected_mesh_faces_torch,
    torch_mesh_depth_geometry_batch,
)


def make_geometry():
    return GeometryParams(
        theta_min=np.deg2rad(-3.0),
        theta_max=np.deg2rad(3.0),
        phi_min=np.deg2rad(-3.0),
        phi_max=np.deg2rad(3.0),
        dtheta=np.deg2rad(0.1),
        dphi=np.deg2rad(0.1),
        width=61,
        height=61,
        max_uv_edge_length=None,
        max_depth_jump=None,
        fill_value=0.0,
    )


def point_from_angles(depth, theta_deg, phi_deg):
    theta = np.deg2rad(theta_deg)
    phi = np.deg2rad(phi_deg)
    rho = depth * np.cos(phi)
    return np.array(
        [
            rho * np.cos(theta),
            rho * np.sin(theta),
            depth * np.sin(phi),
        ],
        dtype=np.float32,
    )


def validate_pixel_centers():
    geom = make_geometry()
    angles = np.deg2rad(np.array([-3.0, 0.0, 3.0]))
    u = (angles - geom.theta_min) / geom.dtheta
    np.testing.assert_allclose(u, [0.0, 30.0, 60.0], atol=1e-10)

    elevations = np.deg2rad(np.array([3.0, 0.0, -3.0]))
    v = (geom.phi_max - elevations) / geom.dphi
    np.testing.assert_allclose(v, [0.0, 30.0, 60.0], atol=1e-10)
    print("Pixel-center convention passed.")


def validate_candidate_face_rule():
    geom = make_geometry()
    # The first face has one vertex inside the exact FOV and must be retained.
    # The second face has all vertices outside and must be dropped.
    vertices = np.stack(
        [
            point_from_angles(10.0, 0.0, 0.0),
            point_from_angles(10.0, 4.0, -1.0),
            point_from_angles(10.0, 4.0, 1.0),
            point_from_angles(12.0, -5.0, -1.0),
            point_from_angles(12.0, -4.0, 1.0),
            point_from_angles(12.0, -4.5, 0.0),
        ]
    )
    triangles = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
    outputs, _ = torch_mesh_depth_geometry_batch(
        P_v=vertices,
        triangles=triangles,
        T=np.eye(4),
        odom_batch=np.eye(4)[None],
        radar_azimuth_batch=np.array([0.0]),
        geom=geom,
        device="cpu",
        with_jacobian=False,
    )
    _, _, stats = outputs[0]
    assert stats["candidate_faces"] == 1, stats
    print("At-least-one-vertex candidate-face rule passed.")


def validate_z_buffer():
    geom = make_geometry()
    # Two identical projected triangles at different radial depths.
    near = np.stack(
        [
            point_from_angles(8.0, -1.0, -1.0),
            point_from_angles(8.0, 1.0, -1.0),
            point_from_angles(8.0, 0.0, 1.0),
        ]
    )
    far = np.stack(
        [
            point_from_angles(12.0, -1.0, -1.0),
            point_from_angles(12.0, 1.0, -1.0),
            point_from_angles(12.0, 0.0, 1.0),
        ]
    )
    vertices = np.concatenate((far, near), axis=0)
    triangles = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
    outputs, _ = torch_mesh_depth_geometry_batch(
        P_v=vertices,
        triangles=triangles,
        T=np.eye(4),
        odom_batch=np.eye(4)[None],
        radar_azimuth_batch=np.array([0.0]),
        geom=geom,
        device="cpu",
        with_jacobian=False,
    )
    D = outputs[0][0]
    np.testing.assert_allclose(D[30, 30], 8.0, atol=2e-3)
    print("Nearest-depth z-buffer passed.")


def validate_barycentric_jacobian():
    geom = make_geometry()
    I_faces = torch.tensor(
        [[[10.0, 10.0, 10.0], [12.0, 50.0, 12.0], [11.0, 28.0, 50.0]]],
        dtype=torch.float64,
    )
    rng = np.random.default_rng(4)
    J_I = torch.tensor(
        rng.normal(scale=0.1, size=(1, 3, 3, 2)),
        dtype=torch.float64,
    )
    D, J_D, _ = _rasterize_projected_mesh_faces_torch(
        I_faces=I_faces,
        J_I_faces=J_I,
        geom=geom,
    )

    h = 1e-5
    fd_columns = []
    common = D.reshape(-1) > 0.0
    for dim in range(2):
        D_plus, _, _ = _rasterize_projected_mesh_faces_torch(
            I_faces=I_faces + h * J_I[..., dim],
            J_I_faces=None,
            geom=geom,
        )
        D_minus, _, _ = _rasterize_projected_mesh_faces_torch(
            I_faces=I_faces - h * J_I[..., dim],
            J_I_faces=None,
            geom=geom,
        )
        common &= (D_plus.reshape(-1) > 0.0) & (D_minus.reshape(-1) > 0.0)
        fd_columns.append(((D_plus - D_minus) / (2.0 * h)).reshape(-1))

    J_fd = torch.stack(fd_columns, dim=1)
    relative_error = (
        torch.linalg.vector_norm(J_D[common] - J_fd[common])
        / torch.linalg.vector_norm(J_fd[common])
    )
    assert float(relative_error) < 1e-6, relative_error
    print(f"Barycentric Jacobian passed: relative error {float(relative_error):.3e}")


def validate_full_pose_jacobian():
    geom = make_geometry()
    vertices = np.stack(
        [
            point_from_angles(10.0, -1.2, -1.0),
            point_from_angles(11.0, 1.2, -0.8),
            point_from_angles(10.5, 0.0, 1.2),
        ]
    )
    triangles = np.array([[0, 1, 2]], dtype=np.int32)

    def evaluate(T, with_jacobian):
        outputs, _ = torch_mesh_depth_geometry_batch(
            P_v=vertices,
            triangles=triangles,
            T=T,
            odom_batch=np.eye(4)[None],
            radar_azimuth_batch=np.array([0.0]),
            geom=geom,
            device="cpu",
            active_dims=range(6),
            with_jacobian=with_jacobian,
        )
        return outputs[0]

    D, J_D, _ = evaluate(np.eye(4), True)
    h = 1e-3
    fd_columns = []
    common = D.reshape(-1) > 0.0
    for dim in range(6):
        perturbation = np.zeros((6, 1))
        perturbation[dim] = h
        D_plus = evaluate(se3op.vec2tran(perturbation), False)[0]
        D_minus = evaluate(se3op.vec2tran(-perturbation), False)[0]
        common &= (D_plus.reshape(-1) > 0.0) & (D_minus.reshape(-1) > 0.0)
        fd_columns.append(((D_plus - D_minus) / (2.0 * h)).reshape(-1))

    J_fd = np.stack(fd_columns, axis=1)
    relative_error = np.linalg.norm(J_D[common] - J_fd[common]) / max(
        np.linalg.norm(J_fd[common]),
        1e-12,
    )
    assert relative_error < 2e-2, relative_error
    print(
        f"Full pose-to-depth Jacobian passed on fixed interior pixels: "
        f"relative error {relative_error:.3e}"
    )


def validate_batched_matches_individual():
    geom = make_geometry()
    vertices = np.stack(
        [
            point_from_angles(8.0, -1.5, -1.0),
            point_from_angles(9.0, 0.2, -1.2),
            point_from_angles(8.5, -0.5, 1.0),
            point_from_angles(11.0, 0.5, -1.0),
            point_from_angles(10.0, 1.8, -0.8),
            point_from_angles(10.5, 1.0, 1.2),
        ]
    )
    triangles = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
    azimuths = np.deg2rad(np.array([-0.5, 0.0, 0.5]))
    odometry = np.repeat(np.eye(4)[None], len(azimuths), axis=0)

    batch_outputs, _ = torch_mesh_depth_geometry_batch(
        P_v=vertices,
        triangles=triangles,
        T=np.eye(4),
        odom_batch=odometry,
        radar_azimuth_batch=azimuths,
        geom=geom,
        device="cpu",
        active_dims=range(6),
        with_jacobian=True,
    )
    for index, azimuth in enumerate(azimuths):
        single_outputs, _ = torch_mesh_depth_geometry_batch(
            P_v=vertices,
            triangles=triangles,
            T=np.eye(4),
            odom_batch=np.eye(4)[None],
            radar_azimuth_batch=np.array([azimuth]),
            geom=geom,
            device="cpu",
            active_dims=range(6),
            with_jacobian=True,
        )
        np.testing.assert_allclose(
            batch_outputs[index][0],
            single_outputs[0][0],
            atol=3e-6,
            rtol=3e-7,
        )
        np.testing.assert_allclose(
            batch_outputs[index][1],
            single_outputs[0][1],
            atol=3e-5,
            rtol=3e-6,
        )
        assert batch_outputs[index][2] == single_outputs[0][2]
    print("Packed multi-azimuth batch matches individual evaluations.")


def main():
    validate_pixel_centers()
    validate_candidate_face_rule()
    validate_z_buffer()
    validate_barycentric_jacobian()
    validate_full_pose_jacobian()
    validate_batched_matches_individual()
    print("All mesh rasterizer checks passed.")


if __name__ == "__main__":
    main()
