import os

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d  # Load before nksr to avoid duplicate pybind registration.
import nksr
import torch
from pyboreas import BoreasDataset
from pylgmath import Transformation
from scipy.spatial import cKDTree

from vtr_pose_graph.graph_iterators import TemporalIterator
import vtr_pose_graph.graph_utils as g_utils
from vtr_utils.bag_file_parsing import Rosbag2GraphFactory
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex


##################
# Helper Functions
##################

def get_submap_vertices(graph_dir):
    factory = Rosbag2GraphFactory(graph_dir)
    test_graph = factory.buildGraph()
    print(
        f"Graph {test_graph} has {test_graph.number_of_vertices} "
        f"vertices and {test_graph.number_of_edges} edges"
    )

    g_utils.set_world_frame(test_graph, test_graph.root)
    v_start = test_graph.get_vertex((0, 0))

    submap_vertices = []
    curr_submap_vid = None
    for vertex, _ in TemporalIterator(v_start):
        map_ptr = vertex.get_data("pointmap_ptr")
        teach_v = test_graph.get_vertex(map_ptr.map_vid)

        if curr_submap_vid != teach_v.id:
            submap_vertices.append(teach_v)
            curr_submap_vid = teach_v.id

    return test_graph, submap_vertices


def color_mesh_by_height(mesh, cmap_name="turbo"):
    """Assign an RGB color to every mesh vertex using its Z coordinate."""
    vertices = np.asarray(mesh.vertices)
    if len(vertices) == 0:
        return

    heights = vertices[:, 2]
    height_min = np.min(heights)
    height_max = np.max(heights)

    if height_max > height_min:
        normalized_height = (heights - height_min) / (height_max - height_min)
    else:
        normalized_height = np.zeros_like(heights)

    colors = plt.get_cmap(cmap_name)(normalized_height)[:, :3]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)


def estimate_point_spacing(points):
    """Return the median nearest-neighbor spacing of the measured points."""
    if len(points) < 2:
        raise ValueError("At least two points are required to estimate spacing.")

    tree = cKDTree(points)
    distances, _ = tree.query(points, k=2, workers=-1)
    nearest = distances[:, 1]
    nearest = nearest[np.isfinite(nearest) & (nearest > 0.0)]
    if len(nearest) == 0:
        raise ValueError("Could not estimate point spacing.")
    return float(np.median(nearest))


def snap_nksr_mesh_to_original_points(
    original_points,
    nksr_vertices,
    nksr_faces,
    spacing,
    max_snap_distance_multiplier=2.5,
    max_edge_length_multiplier=12.0,
    max_centroid_support_multiplier=5.0,
    min_normal_alignment=0.25,
):
    """Transfer NKSR face connectivity onto the measured submap points.

    Every returned triangle indexes ``original_points``. NKSR vertices are used
    only to decide connectivity; they are never retained as depth vertices.
    """
    original_points = np.asarray(original_points, dtype=np.float64)
    nksr_vertices = np.asarray(nksr_vertices, dtype=np.float64)
    nksr_faces = np.asarray(nksr_faces, dtype=np.int64)

    if len(original_points) < 3:
        raise ValueError("At least three original points are required.")
    if len(nksr_faces) == 0:
        raise ValueError("NKSR produced no triangles.")

    point_tree = cKDTree(original_points)
    snap_distances, snapped_indices = point_tree.query(
        nksr_vertices,
        k=1,
        workers=-1,
    )

    mapped_faces = snapped_indices[nksr_faces]
    face_snap_distances = snap_distances[nksr_faces]
    keep = np.max(face_snap_distances, axis=1) <= (
        max_snap_distance_multiplier * spacing
    )
    print(
        "Faces after snap-distance filter: "
        f"{np.count_nonzero(keep)} / {len(nksr_faces)}"
    )

    # Three NKSR vertices may snap onto the same measured point.
    keep &= (
        (mapped_faces[:, 0] != mapped_faces[:, 1])
        & (mapped_faces[:, 1] != mapped_faces[:, 2])
        & (mapped_faces[:, 2] != mapped_faces[:, 0])
    )
    print(f"Faces after collapsed-face filter: {np.count_nonzero(keep)}")

    original_triangles = original_points[mapped_faces]
    edge_01 = original_triangles[:, 1] - original_triangles[:, 0]
    edge_12 = original_triangles[:, 2] - original_triangles[:, 1]
    edge_20 = original_triangles[:, 0] - original_triangles[:, 2]
    edge_lengths = np.stack(
        [
            np.linalg.norm(edge_01, axis=1),
            np.linalg.norm(edge_12, axis=1),
            np.linalg.norm(edge_20, axis=1),
        ],
        axis=1,
    )
    keep &= np.max(edge_lengths, axis=1) <= (
        max_edge_length_multiplier * spacing
    )
    print(f"Faces after edge-length filter: {np.count_nonzero(keep)}")

    original_cross = np.cross(edge_01, -edge_20)
    original_cross_norm = np.linalg.norm(original_cross, axis=1)
    minimum_double_area = max(spacing * spacing * 1e-3, 1e-12)
    keep &= original_cross_norm > minimum_double_area
    print(f"Faces after area filter: {np.count_nonzero(keep)}")

    # Reject mapped triangles whose orientation no longer resembles the NKSR
    # triangle that proposed the connection. Absolute alignment permits an
    # overall normal-sign disagreement while rejecting strongly distorted faces.
    nksr_triangles = nksr_vertices[nksr_faces]
    nksr_cross = np.cross(
        nksr_triangles[:, 1] - nksr_triangles[:, 0],
        nksr_triangles[:, 2] - nksr_triangles[:, 0],
    )
    nksr_cross_norm = np.linalg.norm(nksr_cross, axis=1)
    valid_normals = (
        (original_cross_norm > 1e-12)
        & (nksr_cross_norm > 1e-12)
    )
    normal_alignment = np.zeros(len(nksr_faces), dtype=np.float64)
    normal_alignment[valid_normals] = np.abs(
        np.einsum(
            "ij,ij->i",
            original_cross[valid_normals],
            nksr_cross[valid_normals],
        )
        / (
            original_cross_norm[valid_normals]
            * nksr_cross_norm[valid_normals]
        )
    )
    keep &= normal_alignment >= min_normal_alignment
    print(f"Faces after normal-alignment filter: {np.count_nonzero(keep)}")

    # A snapped triangle can still bridge an unmeasured region. Require its
    # centroid and edge midpoints to remain close to some measured point.
    support_samples = np.stack(
        [
            np.mean(original_triangles, axis=1),
            0.5 * (original_triangles[:, 0] + original_triangles[:, 1]),
            0.5 * (original_triangles[:, 1] + original_triangles[:, 2]),
            0.5 * (original_triangles[:, 2] + original_triangles[:, 0]),
        ],
        axis=1,
    )
    support_distances, _ = point_tree.query(
        support_samples.reshape(-1, 3),
        k=1,
        workers=-1,
    )
    support_distances = support_distances.reshape(-1, 4)
    keep &= np.max(support_distances, axis=1) <= (
        max_centroid_support_multiplier * spacing
    )
    print(f"Faces after measured-support filter: {np.count_nonzero(keep)}")

    mapped_faces = mapped_faces[keep]
    if len(mapped_faces) == 0:
        raise RuntimeError(
            "All snapped NKSR triangles were rejected. Increase the snap, edge, "
            "or support thresholds and inspect the NKSR reconstruction."
        )

    # Deduplicate connectivity independent of winding while retaining the first
    # face's winding for visualization and later rasterization.
    canonical_faces = np.sort(mapped_faces, axis=1)
    _, unique_indices = np.unique(canonical_faces, axis=0, return_index=True)
    mapped_faces = mapped_faces[np.sort(unique_indices)]
    print(f"Faces after duplicate removal: {len(mapped_faces)}")

    mesh = o3d.geometry.TriangleMesh(
        vertices=o3d.utility.Vector3dVector(original_points),
        triangles=o3d.utility.Vector3iVector(mapped_faces),
    )
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()

    print(
        f"Snapped mesh: {len(mesh.vertices)} original vertices available, "
        f"{len(mesh.triangles)} retained triangles, "
        f"{len(np.unique(np.asarray(mesh.triangles)))} referenced vertices"
    )
    return mesh


def reconstruct_nksr_with_original_vertices(
    pcd,
    reconstructor,
    device,
    detail_level=1.0,
    mise_iter=1,
    trim=True,
    max_snap_distance_multiplier=2.5,
    max_edge_length_multiplier=12.0,
    max_centroid_support_multiplier=5.0,
    min_normal_alignment=0.25,
):
    """Run NKSR and transfer its triangle connectivity to measured points."""
    original_points = np.asarray(pcd.points)
    if len(original_points) < 3:
        raise ValueError("NKSR reconstruction requires at least three points.")

    spacing = estimate_point_spacing(original_points)
    normal_radius = max(4.0 * spacing, 0.20)
    print(f"Filtered original points: {len(original_points)}")
    print(f"Median nearest-neighbor spacing: {spacing:.4f} m")
    print(f"Normal estimation radius: {normal_radius:.4f} m")

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius,
            max_nn=50,
        )
    )
    pcd.orient_normals_consistent_tangent_plane(k=20)
    pcd.normalize_normals()

    xyz = torch.from_numpy(original_points).float().to(device)
    normals = torch.from_numpy(np.asarray(pcd.normals)).float().to(device)

    field = reconstructor.reconstruct(
        xyz,
        normal=normals,
        detail_level=detail_level,
    )
    if field is None:
        raise RuntimeError("NKSR reconstruction returned no field.")

    nksr_mesh = field.extract_dual_mesh(mise_iter=mise_iter, trim=trim)
    nksr_vertices = nksr_mesh.v.detach().cpu().numpy()
    nksr_faces = nksr_mesh.f.detach().cpu().numpy()
    print(
        f"Raw NKSR mesh: {len(nksr_vertices)} synthetic vertices, "
        f"{len(nksr_faces)} triangles"
    )

    return snap_nksr_mesh_to_original_points(
        original_points=original_points,
        nksr_vertices=nksr_vertices,
        nksr_faces=nksr_faces,
        spacing=spacing,
        max_snap_distance_multiplier=max_snap_distance_multiplier,
        max_edge_length_multiplier=max_edge_length_multiplier,
        max_centroid_support_multiplier=max_centroid_support_multiplier,
        min_normal_alignment=min_normal_alignment,
    )


######################
# Dataset and settings
######################

boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
boreas_data = os.getenv("VTRRDATA")
if not boreas_vtr_wrapper_dir:
    raise RuntimeError("VTRROOT is not set.")
if not boreas_data:
    raise RuntimeError("VTRRDATA is not set.")

lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")
bd = BoreasDataset(boreas_data)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"NKSR device: {device}")
reconstructor = nksr.Reconstructor(device)

radar_start_frame = 158 # 100
radar_end_frame = None
visualize_once = True

# Connectivity-transfer tuning. All distance thresholds are multiples of the
# median nearest-neighbor spacing in the filtered measured point cloud.
# max_snap_distance_multiplier = 5.0
# max_edge_length_multiplier = 12.0
# max_centroid_support_multiplier = 5.0
# min_normal_alignment = 0.25
max_snap_distance_multiplier = 10.0
max_edge_length_multiplier = 20.0
max_centroid_support_multiplier = 7.0
min_normal_alignment = 0.1


################
# Sequence loop
################

for seq in bd.sequences:
    print(f"SequenceID: {seq.ID}")
    if seq.ID != "boreas-2024-12-03-12-54":
        continue
    if radar_end_frame is None:
        radar_end_frame = len(seq.radar_frames) - 1

    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")
    test_graph, submap_vertices = get_submap_vertices(graph_dir=graph_dir)

    T_wheel_robot_np = np.array(
        [
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    T_wheel_robot = Transformation(T_ba=T_wheel_robot_np)
    T_applanix_wheel = Transformation(T_ba=seq.calib.T_applanix_wheel)
    T_applanix_lidar = Transformation(T_ba=seq.calib.T_applanix_lidar)
    T_lidar_wheel = T_applanix_lidar.inverse() * T_applanix_wheel
    T_lidar_robot = T_lidar_wheel * T_wheel_robot

    lidar_frame_idx = 0
    submap_vertices_idx = 0
    radar_frame_idx = radar_start_frame
    print("Number of Submaps:", len(submap_vertices))

    while (
        submap_vertices_idx < len(submap_vertices) - 1
        and lidar_frame_idx < len(seq.lidar_frames)
        and radar_frame_idx <= radar_end_frame
    ):
        curr_submap = submap_vertices[submap_vertices_idx]
        next_submap = submap_vertices[submap_vertices_idx + 1]
        radar_frame = seq.radar_frames[radar_frame_idx]
        lidar_frame = seq.lidar_frames[lidar_frame_idx]

        if int(radar_frame.frame) < curr_submap.stamp // 1000:
            radar_frame_idx += 1
            continue

        if next_submap.stamp // 1000 < int(radar_frame.frame):
            submap_vertices_idx += 1
            continue

        if int(lidar_frame.frame) != curr_submap.stamp // 1000:
            lidar_frame_idx += 1
            continue

        map_pts = extract_points_from_vertex(curr_submap, msg="pointmap")
        print("-" * 60)
        print(f"Radar frame: {radar_frame.frame}")
        print(f"Radar frame index: {radar_frame_idx}")

        map_pts = convert_points_to_frame(map_pts, T_lidar_robot)

        T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
        T_enu_radar = Transformation(T_ba=radar_frame.pose)
        map_pts = convert_points_to_frame(
            map_pts,
            T_enu_radar.inverse() * T_enu_lidar,
        )

        finite_mask = np.all(np.isfinite(map_pts), axis=0)
        map_pts = map_pts[:, finite_mask]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(map_pts.T)
        pcd, _ = pcd.remove_radius_outlier(
            nb_points=3,
            radius=0.50,
        )
        radar_frame.unload_data()

        mesh = reconstruct_nksr_with_original_vertices(
            pcd=pcd,
            reconstructor=reconstructor,
            device=device,
            detail_level=1.0,
            mise_iter=1,
            trim=True,
            max_snap_distance_multiplier=max_snap_distance_multiplier,
            max_edge_length_multiplier=max_edge_length_multiplier,
            max_centroid_support_multiplier=max_centroid_support_multiplier,
            min_normal_alignment=min_normal_alignment,
        )

        color_mesh_by_height(mesh, cmap_name="jet")
        pcd.paint_uniform_color([0.10, 0.10, 0.10])

        o3d.visualization.draw_geometries(
            [mesh, pcd],
            window_name=f"NKSR connectivity on original points {radar_frame.frame}",
            mesh_show_back_face=True,
        )

        radar_frame_idx += 1
        if visualize_once:
            break

    break
