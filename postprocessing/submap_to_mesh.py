import argparse
import os
from pathlib import Path
import time
import gc

import matplotlib.pyplot as plt
import nksr
import numpy as np
import open3d as o3d
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


def build_lidar_to_robot_transform(seq):
    """Return the static transform that maps robot-frame points into LiDAR."""
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
    return T_lidar_wheel * T_wheel_robot


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


def color_mesh_by_height(mesh, cmap_name="turbo"):
    """Assign mesh vertex colors from the robot-frame Z coordinate."""
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


def snap_nksr_mesh_to_original_points(
    original_points,
    nksr_vertices,
    nksr_faces,
    spacing,
    output_vertices=None,
    max_snap_distance_multiplier=2.5,
    max_edge_length_multiplier=12.0,
    max_centroid_support_multiplier=5.0,
    min_normal_alignment=0.25,
):
    """Transfer NKSR face connectivity onto the measured submap points.

    Every returned triangle indexes ``original_points``. NKSR vertices are used
    only to decide connectivity; they are never retained as depth vertices.
    ``output_vertices`` may contain the same ordered measured points expressed
    in another rigid frame, allowing reconstruction in LiDAR coordinates while
    saving the final mesh in the submap/robot frame.
    """
    original_points = np.asarray(original_points, dtype=np.float64)
    if output_vertices is None:
        output_vertices = original_points
    output_vertices = np.asarray(output_vertices, dtype=np.float64)
    nksr_vertices = np.asarray(nksr_vertices, dtype=np.float64)
    nksr_faces = np.asarray(nksr_faces, dtype=np.int64)

    if len(original_points) < 3:
        raise ValueError("At least three original points are required.")
    if output_vertices.shape != original_points.shape:
        raise ValueError(
            "output_vertices must contain the same ordered points as "
            "original_points."
        )
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
        vertices=o3d.utility.Vector3dVector(output_vertices),
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
    output_vertices=None,
    detail_level=1.0,
    mise_iter=1,
    trim=True,
    max_snap_distance_multiplier=2.5,
    max_edge_length_multiplier=12.0,
    max_centroid_support_multiplier=5.0,
    min_normal_alignment=0.25,
):
    """Run NKSR and transfer its connectivity to ordered measured vertices."""
    original_points = np.asarray(pcd.points)
    if len(original_points) < 3:
        raise ValueError("NKSR reconstruction requires at least three points.")
    if output_vertices is not None:
        output_vertices = np.asarray(output_vertices)
        if output_vertices.shape != original_points.shape:
            raise ValueError(
                "output_vertices must match the reconstruction point shape."
            )

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

    mesh = snap_nksr_mesh_to_original_points(
        original_points=original_points,
        nksr_vertices=nksr_vertices,
        nksr_faces=nksr_faces,
        spacing=spacing,
        output_vertices=output_vertices,
        max_snap_distance_multiplier=max_snap_distance_multiplier,
        max_edge_length_multiplier=max_edge_length_multiplier,
        max_centroid_support_multiplier=max_centroid_support_multiplier,
        min_normal_alignment=min_normal_alignment,
    )

    # NKSR fields and extracted meshes can own substantial CUDA allocations
    # outside PyTorch's normal tensor accounting. Release them before starting
    # reconstruction of the next submap.
    del nksr_mesh
    del field
    del xyz
    del normals
    gc.collect()
    if device.type == "cuda":
        torch.cuda.empty_cache()

    return mesh


def save_submap_mesh(
    mesh,
    output_dir,
    sequence_id,
    submap_stamp_ns,
    submap_vertex_id,
    reconstruction_parameters,
):
    """Save a submap mesh as memory-mappable NumPy arrays."""
    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    triangles = np.asarray(mesh.triangles, dtype=np.int32)

    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError("Refusing to save an empty submap mesh.")
    if np.min(triangles) < 0 or np.max(triangles) >= len(vertices):
        raise ValueError("Triangle indices do not reference the saved vertex array.")

    # Store only measured vertices referenced by retained triangles. Connectivity
    # is remapped into the compact array, and source_point_indices preserves the
    # corresponding indices in the filtered original point cloud.
    source_point_indices = np.unique(triangles)
    compact_index = np.full(len(vertices), -1, dtype=np.int32)
    compact_index[source_point_indices] = np.arange(
        len(source_point_indices),
        dtype=np.int32,
    )
    vertices = vertices[source_point_indices]
    triangles = compact_index[triangles]

    output_dir.mkdir(parents=True, exist_ok=True)
    np.save(output_dir / "vertices.npy", vertices)
    np.save(output_dir / "triangles.npy", triangles)
    np.save(
        output_dir / "source_point_indices.npy",
        source_point_indices.astype(np.int32),
    )
    np.savez(
        output_dir / "metadata.npz",
        coordinate_frame=np.asarray("submap_robot"),
        format_version=np.asarray(2, dtype=np.int32),
        sequence_id=np.asarray(sequence_id),
        submap_stamp_ns=np.asarray(submap_stamp_ns, dtype=np.int64),
        submap_stamp_us=np.asarray(submap_stamp_ns // 1000, dtype=np.int64),
        submap_vertex_id=np.asarray(str(submap_vertex_id)),
        max_snap_distance_multiplier=np.asarray(
            reconstruction_parameters["max_snap_distance_multiplier"],
            dtype=np.float64,
        ),
        max_edge_length_multiplier=np.asarray(
            reconstruction_parameters["max_edge_length_multiplier"],
            dtype=np.float64,
        ),
        max_centroid_support_multiplier=np.asarray(
            reconstruction_parameters["max_centroid_support_multiplier"],
            dtype=np.float64,
        ),
        min_normal_alignment=np.asarray(
            reconstruction_parameters["min_normal_alignment"],
            dtype=np.float64,
        ),
    )
    print(
        f"Saved {len(vertices)} referenced original vertices and "
        f"{len(triangles)} triangles to {output_dir}"
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
parser = argparse.ArgumentParser(description="Generate submap meshes for a Boreas sequence.")
parser.add_argument("--sequence-id", required=True)
args = parser.parse_args()
bd = BoreasDataset(boreas_data, [[args.sequence_id]])

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"NKSR device: {device}")
reconstructor = nksr.Reconstructor(device)

output_root = Path(__file__).resolve().parent / "submap_meshes"
overwrite_existing = True
visualize_meshes = True
visualization_frame_time_s = 0.10

visualization_paused = False
visualizer_open = False
visualizer = None
display_mesh = None
display_mesh_added = False


def toggle_visualization_pause(_visualizer):
    """Toggle whether visualization blocks mesh generation."""
    global visualization_paused
    visualization_paused = not visualization_paused
    print(f"Visualization {'paused' if visualization_paused else 'resumed'}")
    return False


if visualize_meshes:
    visualizer = o3d.visualization.VisualizerWithKeyCallback()
    visualizer.register_key_callback(ord(" "), toggle_visualization_pause)
    visualizer_open = visualizer.create_window(
        window_name="NKSR submap meshes — Space: pause/resume",
    )
    if visualizer_open:
        display_mesh = o3d.geometry.TriangleMesh()
        render_options = visualizer.get_render_option()
        render_options.mesh_show_back_face = True
        visualizer.poll_events()
        visualizer.update_renderer()
    else:
        print("Could not create the Open3D window; continuing without visualization.")

# Connectivity-transfer tuning. All distance thresholds are multiples of the
# median nearest-neighbor spacing in the filtered measured point cloud.
max_snap_distance_multiplier = 10.0
max_edge_length_multiplier = 25.0
max_centroid_support_multiplier = 20.0
min_normal_alignment = 0.0
reconstruction_parameters = {
    "max_snap_distance_multiplier": max_snap_distance_multiplier,
    "max_edge_length_multiplier": max_edge_length_multiplier,
    "max_centroid_support_multiplier": max_centroid_support_multiplier,
    "min_normal_alignment": min_normal_alignment,
}


################
# Sequence loop
################

for seq in bd.sequences:
    print(f"SequenceID: {seq.ID}")

    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")
    test_graph, submap_vertices = get_submap_vertices(graph_dir=graph_dir)
    T_lidar_robot = build_lidar_to_robot_transform(seq)

    sequence_output_dir = output_root / seq.ID
    sequence_output_dir.mkdir(parents=True, exist_ok=True)
    print("Number of Submaps:", len(submap_vertices))
    print(f"Mesh output directory: {sequence_output_dir}")

    for submap in submap_vertices:
        submap_stamp_us = submap.stamp // 1000
        output_dir = sequence_output_dir / str(submap_stamp_us)
        required_output_files = (
            output_dir / "vertices.npy",
            output_dir / "triangles.npy",
            output_dir / "source_point_indices.npy",
            output_dir / "metadata.npz",
        )
        output_is_complete = all(path.is_file() for path in required_output_files)
        if output_is_complete and not overwrite_existing:
            print(f"Skipping existing mesh: {output_dir}")
            continue

        print("-" * 60)
        print(f"Submap stamp: {submap_stamp_us}")
        print(f"Submap vertex ID: {submap.id}")

        # Filter once in the native submap/robot frame, then transform those
        # exact ordered points into LiDAR coordinates for NKSR. The resulting
        # triangle indices therefore also index the robot-frame point array.
        map_pts_robot = extract_points_from_vertex(submap, msg="pointmap")
        finite_mask = np.all(np.isfinite(map_pts_robot), axis=0)
        map_pts_robot = map_pts_robot[:, finite_mask]

        pcd_robot = o3d.geometry.PointCloud()
        pcd_robot.points = o3d.utility.Vector3dVector(map_pts_robot.T)
        # pcd_robot, _ = pcd_robot.remove_radius_outlier(
        #     nb_points=3,
        #     radius=0.50,
        # )
        filtered_points_robot = np.asarray(pcd_robot.points).copy()
        filtered_points_lidar = convert_points_to_frame(
            filtered_points_robot.T,
            T_lidar_robot,
        ).T

        pcd_lidar = o3d.geometry.PointCloud()
        pcd_lidar.points = o3d.utility.Vector3dVector(filtered_points_lidar)

        mesh = reconstruct_nksr_with_original_vertices(
            pcd=pcd_lidar,
            reconstructor=reconstructor,
            device=device,
            output_vertices=filtered_points_robot,
            detail_level=0.0,
            mise_iter=1,
            trim=True,
            max_snap_distance_multiplier=max_snap_distance_multiplier,
            max_edge_length_multiplier=max_edge_length_multiplier,
            max_centroid_support_multiplier=max_centroid_support_multiplier,
            min_normal_alignment=min_normal_alignment,
        )

        save_submap_mesh(
            mesh=mesh,
            output_dir=output_dir,
            sequence_id=seq.ID,
            submap_stamp_ns=submap.stamp,
            submap_vertex_id=submap.id,
            reconstruction_parameters=reconstruction_parameters,
        )

        if visualize_meshes and visualizer_open:
            color_mesh_by_height(mesh, cmap_name="jet")

            # Keep one registered geometry object and replace its buffers. This
            # preserves the camera and avoids opening a blocking window for each
            # submap.
            display_mesh.vertices = o3d.utility.Vector3dVector(
                np.asarray(mesh.vertices).copy()
            )
            display_mesh.triangles = o3d.utility.Vector3iVector(
                np.asarray(mesh.triangles).copy()
            )
            display_mesh.vertex_normals = o3d.utility.Vector3dVector(
                np.asarray(mesh.vertex_normals).copy()
            )
            display_mesh.triangle_normals = o3d.utility.Vector3dVector(
                np.asarray(mesh.triangle_normals).copy()
            )
            display_mesh.vertex_colors = o3d.utility.Vector3dVector(
                np.asarray(mesh.vertex_colors).copy()
            )

            if not display_mesh_added:
                visualizer.add_geometry(display_mesh)
                display_mesh_added = True
            else:
                visualizer.update_geometry(display_mesh)

            print(
                f"Displaying submap {submap_stamp_us}. "
                "Press Space to pause/resume."
            )
            display_deadline = time.monotonic() + visualization_frame_time_s
            while visualization_paused or time.monotonic() < display_deadline:
                if not visualizer.poll_events():
                    visualizer_open = False
                    print(
                        "Open3D window closed; continuing mesh generation "
                        "without visualization."
                    )
                    break
                visualizer.update_renderer()

        # The display mesh owns its own copied CPU buffers, so the reconstruction
        # mesh and point cloud can be released immediately after this submap.
        del mesh
        del pcd_lidar
        del pcd_robot
        del filtered_points_lidar
        del filtered_points_robot
        gc.collect()
        if device.type == "cuda":
            torch.cuda.empty_cache()

    break

if visualizer is not None and visualizer_open:
    visualizer.destroy_window()
