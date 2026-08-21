from pathlib import Path
from time import perf_counter

import numpy as np
import torch
from pylgmath import Transformation
from scipy.ndimage import gaussian_filter1d, shift
from vtr_pose_graph.graph_iterators import TemporalIterator
from vtr_pose_graph import graph_utils as g_utils
from vtr_utils.bag_file_parsing import Rosbag2GraphFactory
from vtr_utils.plot_utils import convert_points_to_frame


def get_submap_vertices(graph_dir):
    factory = Rosbag2GraphFactory(graph_dir)
    graph = factory.buildGraph()
    print(f"Graph {graph} {graph.number_of_vertices} vertices and {graph.number_of_edges} edges")

    g_utils.set_world_frame(graph, graph.root)
    v_start = graph.get_vertex((0, 0))

    submap_vertices = []
    curr_submap_vid = None
    for vertex, _ in TemporalIterator(v_start):
        map_ptr = vertex.get_data("pointmap_ptr")
        teach_v = graph.get_vertex(map_ptr.map_vid)
        if curr_submap_vid != teach_v.id:
            submap_vertices.append(teach_v)
            curr_submap_vid = teach_v.id

    return graph, submap_vertices


def get_path_vertices_with_submaps(graph_dir):
    factory = Rosbag2GraphFactory(graph_dir)
    graph = factory.buildGraph()
    print(f"Graph {graph} {graph.number_of_vertices} vertices and {graph.number_of_edges} edges")

    g_utils.set_world_frame(graph, graph.root)
    v_start = graph.get_vertex((0, 0))

    candidates = []
    for vertex, _ in TemporalIterator(v_start):
        map_ptr = vertex.get_data("pointmap_ptr")
        candidates.append((vertex, graph.get_vertex(map_ptr.map_vid)))

    return graph, candidates


def build_patch_config(
    fov_deg,
    res_deg,
    min_range=0.0,
    max_uv_edge_length=None,
    max_depth_jump=2.0,
):
    hfov = np.deg2rad(fov_deg)
    vfov = np.deg2rad(fov_deg)
    dtheta = np.deg2rad(res_deg)
    dphi = np.deg2rad(res_deg)
    width = int(np.round(hfov / dtheta)) + 1
    height = int(np.round(vfov / dphi)) + 1
    if width % 2 != 1 or height % 2 != 1:
        raise ValueError("Patch dimensions must be odd so there is an exact center pixel.")

    uu, vv = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    query_uv = np.stack([uu.ravel(), vv.ravel()], axis=1)

    return {
        "hfov": hfov,
        "vfov": vfov,
        "theta_min": -0.5 * hfov,
        "theta_max": 0.5 * hfov,
        "phi_min": -0.5 * vfov,
        "phi_max": 0.5 * vfov,
        "dtheta": dtheta,
        "dphi": dphi,
        "width": width,
        "height": height,
        "query_uv": query_uv,
        "min_range": min_range,
        "max_uv_edge_length": max_uv_edge_length,
        "max_depth_jump": max_depth_jump,
        "fill_value": 0.0,
    }


def correct_offsets(radar_frame, body_velocity, radar_offset):
    body_velocity = np.asarray(body_velocity)
    if body_velocity.shape != (2,) or not np.isfinite(body_velocity).all():
        raise ValueError(f"Radar body velocity must be a finite [vx, vy], got {body_velocity}.")

    shifted_polar = shift(
        radar_frame.polar,
        shift=(0, radar_offset / radar_frame.resolution),
        order=3,
        mode="nearest",
    )

    vx, vy = -body_velocity
    u = vx * np.cos(radar_frame.azimuths) + vy * np.sin(radar_frame.azimuths)
    # delta_r_d = seq.calib.radar_doppler_beta * u
    delta_r_d = 0.05024 * u
    chirp_sign = np.where(radar_frame.chirp_type == 0, -1, radar_frame.chirp_type)
    doppler_shift = chirp_sign * delta_r_d / radar_frame.resolution

    corrected = np.empty_like(shifted_polar)
    for i in range(shifted_polar.shape[0]):
        corrected[i] = shift(
            shifted_polar[i],
            shift=-doppler_shift[i],
            order=3,
            mode="nearest",
        )
    return corrected


def cen_filter_2d(
    polar_image,
    sigma_gauss=15.0,
    z_q=2.5,
    noise_scale=0.5,
    output_width=None,
):
    mean_val = np.mean(polar_image, axis=1, keepdims=True)
    q = polar_image - mean_val
    if output_width is None:
        output_width = q.shape[1]
    elif (
        not isinstance(output_width, (int, np.integer))
        or not 0 < output_width <= q.shape[1]
    ):
        raise ValueError(f"output_width must be in [1, {q.shape[1]}], got {output_width}.")

    filter_radius = int(4.0 * sigma_gauss + 0.5)
    filter_input_width = min(output_width + filter_radius, q.shape[1])
    p = gaussian_filter1d(
        q[:, :filter_input_width], sigma=sigma_gauss, axis=1, mode="reflect"
    )[:, :output_width]

    neg_mask = q < 0
    count = np.sum(neg_mask, axis=1, keepdims=True)
    q_neg_sq = (q * neg_mask) ** 2
    sum_q_neg_sq = np.sum(q_neg_sq, axis=1, keepdims=True)
    sigma_q_sq = np.divide(2.0 * sum_q_neg_sq, count, out=np.zeros_like(count, dtype=float), where=count != 0)
    sigma_q = noise_scale * np.sqrt(sigma_q_sq)
    sigma_q[count == 0] = 0.034

    threshold = z_q * sigma_q
    q = q[:, :output_width]

    eps = 1e-8
    pow_p = (p / (sigma_q + eps)) ** 2
    pow_qp = ((q - p) / (sigma_q + eps)) ** 2

    npp = np.exp(-0.5 * pow_p)
    nqp = np.exp(-0.5 * pow_qp)

    y = q * (1.0 - nqp) + p * (nqp - npp)
    y[y <= threshold] = 0.0
    return y


def build_T_lidar_robot(seq):
    T_wheel_robot_np = np.array([
        [0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    T_wheel_robot = Transformation(T_ba=T_wheel_robot_np)
    T_applanix_wheel = Transformation(T_ba=seq.calib.T_applanix_wheel)
    T_applanix_lidar = Transformation(T_ba=seq.calib.T_applanix_lidar)
    T_lidar_wheel = T_applanix_lidar.inverse() * T_applanix_wheel
    return T_lidar_wheel * T_wheel_robot


def load_submap_mesh_to_enu(mesh_root, sequence_id, submap, T_lidar_robot, lidar_pose, device):
    submap_stamp_us = submap.stamp // 1000
    mesh_dir = Path(mesh_root) / sequence_id / str(submap_stamp_us)
    vertices_path = mesh_dir / "vertices.npy"
    triangles_path = mesh_dir / "triangles.npy"
    metadata_path = mesh_dir / "metadata.npz"

    missing = [path for path in (vertices_path, triangles_path, metadata_path) if not path.is_file()]
    if missing:
        missing_text = ", ".join(str(path) for path in missing)
        raise FileNotFoundError(f"Saved mesh submap {submap_stamp_us} is incomplete: {missing_text}")

    t0 = perf_counter()
    vertices_robot = np.load(vertices_path)
    triangles = np.load(triangles_path)
    with np.load(metadata_path) as metadata:
        coordinate_frame = str(metadata["coordinate_frame"].item())
        metadata_stamp_us = int(metadata["submap_stamp_us"].item())
    if coordinate_frame != "submap_robot":
        raise ValueError(f"Expected mesh coordinate frame 'submap_robot', got {coordinate_frame!r}.")
    if metadata_stamp_us != submap_stamp_us:
        raise ValueError(f"Mesh metadata stamp {metadata_stamp_us} does not match submap stamp {submap_stamp_us}.")
    if vertices_robot.ndim != 2 or vertices_robot.shape[1] != 3:
        raise ValueError(f"Mesh vertices must have shape (N, 3), got {vertices_robot.shape}.")
    if triangles.ndim != 2 or triangles.shape[1] != 3:
        raise ValueError(f"Mesh triangles must have shape (F, 3), got {triangles.shape}.")
    if len(triangles) == 0 or np.min(triangles) < 0 or np.max(triangles) >= len(vertices_robot):
        raise ValueError("Mesh triangles do not validly index saved vertices.")
    disk_load_time = perf_counter() - t0

    t0 = perf_counter()
    vertices_lidar = convert_points_to_frame(vertices_robot.T, T_lidar_robot)
    T_enu_lidar = Transformation(T_ba=lidar_pose)
    vertices_enu = convert_points_to_frame(vertices_lidar, T_enu_lidar)
    vertices_gpu = torch.as_tensor(
        vertices_enu.T, device=device, dtype=torch.float32
    ).contiguous()
    triangles_gpu = torch.as_tensor(
        triangles, device=device, dtype=torch.int32
    ).contiguous()
    if torch.device(device).type == "cuda":
        torch.cuda.synchronize(device)
    transform_upload_time = perf_counter() - t0

    print(f"Loaded submap mesh {submap_stamp_us}: {len(vertices_robot)} vertices, {len(triangles)} faces")
    return vertices_gpu, triangles_gpu, {
        "mesh_disk_load": disk_load_time,
        "mesh_robot_to_enu_upload": transform_upload_time,
    }
