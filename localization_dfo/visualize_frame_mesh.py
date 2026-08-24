import argparse
import os
from pathlib import Path

import numpy as np
import open3d as o3d
from pyboreas import BoreasDataset

from localization_dfo.io_utils import (
    build_T_lidar_robot,
    get_path_vertices_with_submaps,
    load_submap_mesh_to_enu,
)
from localization_dfo.pipeline_dfo import (
    build_T_radar_robot,
    build_path_candidates,
    nearest_submap_idx,
)


def main():
    parser = argparse.ArgumentParser(
        description="Inspect a localization submap in a GT radar frame."
    )
    parser.add_argument("--map-sequence", required=True)
    parser.add_argument("--loc-sequence", required=True)
    parser.add_argument("--frame", type=int, required=True)
    parser.add_argument("--crop-radius", type=float, default=120.0)
    parser.add_argument("--near-radius", type=float, default=3.0)
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path("postprocessing/submap_meshes"),
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--screenshots", action="store_true")
    args = parser.parse_args()

    root = Path(os.environ["VTRROOT"])
    data_root = os.environ["VTRRDATA"]
    results_root = Path(os.environ.get("VTRRESULT", root / "results"))
    dataset = BoreasDataset(
        data_root,
        split=[
            [sequence_id]
            for sequence_id in dict.fromkeys((args.map_sequence, args.loc_sequence))
        ],
    )
    map_sequence = dataset.get_seq_from_ID(args.map_sequence)
    loc_sequence = dataset.get_seq_from_ID(args.loc_sequence)
    radar_frame = loc_sequence.get_radar(args.frame)
    T_radar_enu = np.linalg.inv(radar_frame.pose)

    graph_dir = (
        results_root
        / "lidar"
        / args.map_sequence
        / args.map_sequence
        / "graph"
    )
    _, pairs = get_path_vertices_with_submaps(str(graph_dir))
    T_lidar_robot = build_T_lidar_robot(map_sequence)
    candidates = build_path_candidates(map_sequence, pairs, T_lidar_robot)
    T_robot_radar = np.linalg.inv(
        build_T_radar_robot(loc_sequence, build_T_lidar_robot(loc_sequence))
    )
    submap_idx = nearest_submap_idx(T_robot_radar @ T_radar_enu, candidates)
    submap, lidar_frame, _ = candidates[submap_idx]

    vertices, triangles, _ = load_submap_mesh_to_enu(
        args.mesh_root if args.mesh_root.is_absolute() else root / args.mesh_root,
        args.map_sequence,
        submap,
        T_lidar_robot,
        lidar_frame.pose,
        "cpu",
    )
    vertices = vertices.numpy()
    triangles = triangles.numpy()
    vertices_radar = vertices @ T_radar_enu[:3, :3].T + T_radar_enu[:3, 3]

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vertices_radar),
        o3d.utility.Vector3iVector(triangles),
    )
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.55, 0.55, 0.55])
    crop = o3d.geometry.AxisAlignedBoundingBox(
        [-args.crop_radius] * 3, [args.crop_radius] * 3
    )
    mesh = mesh.crop(crop)

    cropped_vertices = np.asarray(mesh.vertices)
    distances = np.linalg.norm(cropped_vertices, axis=1)
    colors = np.full((len(cropped_vertices), 3), [0.55, 0.55, 0.55])
    colors[distances <= args.near_radius] = [1.0, 0.1, 0.0]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    near = mesh.crop(
        o3d.geometry.AxisAlignedBoundingBox(
            [-args.near_radius] * 3, [args.near_radius] * 3
        )
    )
    near.paint_uniform_color([1.0, 0.1, 0.0])
    near.compute_vertex_normals()
    near_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(near)
    near_wireframe.paint_uniform_color([1.0, 0.8, 0.0])

    origin = o3d.geometry.TriangleMesh.create_sphere(radius=0.12)
    origin.paint_uniform_color([0.0, 1.0, 1.0])
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)

    args.output.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(args.output / "mesh_gt_radar_frame.ply"), mesh)
    o3d.io.write_triangle_mesh(str(args.output / "near_mesh_gt_radar_frame.ply"), near)
    o3d.io.write_line_set(str(args.output / "near_mesh_wireframe.ply"), near_wireframe)

    if args.screenshots:
        visualizer = o3d.visualization.Visualizer()
        visualizer.create_window(width=1400, height=900, visible=False)
        for name, geometries, zoom in (
            ("near_field.png", [near_wireframe, origin, axes], 0.7),
            ("context.png", [mesh, near_wireframe, origin, axes], 0.65),
        ):
            visualizer.clear_geometries()
            for index, geometry in enumerate(geometries):
                visualizer.add_geometry(geometry, reset_bounding_box=index == 0)
            visualizer.get_render_option().mesh_show_back_face = True
            view = visualizer.get_view_control()
            view.set_lookat([0.0, 0.0, 0.0])
            view.set_front([0.7, -0.7, -0.5])
            view.set_up([0.0, 0.0, 1.0])
            view.set_zoom(zoom)
            visualizer.poll_events()
            visualizer.update_renderer()
            visualizer.capture_screen_image(str(args.output / name), do_render=True)
        visualizer.destroy_window()

    original_distances = np.linalg.norm(vertices_radar, axis=1)
    print(f"map sequence: {args.map_sequence}")
    print(f"localization sequence: {args.loc_sequence}")
    print(f"frame: {args.frame}")
    print(f"submap timestamp: {submap.stamp // 1000}")
    print(f"closest mesh vertex to GT radar: {original_distances.min():.3f} m")
    print(
        f"vertices within {args.near_radius:g} m: "
        f"{np.count_nonzero(original_distances <= args.near_radius)}"
    )
    print(f"wrote: {args.output}")

    if not args.no_window:
        o3d.visualization.draw_geometries(
            [mesh, near_wireframe, origin, axes],
            window_name=f"Frame {args.frame}: mesh in GT radar frame",
            width=1400,
            height=900,
            mesh_show_back_face=True,
        )


if __name__ == "__main__":
    main()
