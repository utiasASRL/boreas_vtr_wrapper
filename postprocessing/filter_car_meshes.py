#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
from pyboreas import BoreasDataset
from pyboreas.data.bounding_boxes import BoundingBoxes
from sensor_msgs_py.point_cloud2 import read_points
from vtr_pose_graph.graph_iterators import TemporalIterator
import vtr_pose_graph.graph_utils as g_utils
from vtr_utils.bag_file_parsing import Rosbag2GraphFactory

from localization_dfo.io_utils import build_T_lidar_robot


def get_submaps(graph_dir):
    graph = Rosbag2GraphFactory(str(graph_dir)).buildGraph()
    g_utils.set_world_frame(graph, graph.root)
    submaps = {}
    for vertex, _ in TemporalIterator(graph.get_vertex((0, 0))):
        submap = graph.get_vertex(vertex.get_data("pointmap_ptr").map_vid)
        submaps[submap.stamp // 1000] = submap
    return graph, submaps


def decode_point_timestamps(pointmap_msg):
    points = read_points(pointmap_msg.point_cloud)
    offset = points.dtype.fields["flex21"][1]
    timestamps = np.ndarray(
        points.shape,
        dtype=np.int64,
        buffer=points,
        offset=offset,
        strides=(points.dtype.itemsize,),
    )
    finite = np.isfinite(
        np.column_stack((points["x"], points["y"], points["z"]))
    ).all(axis=1)
    return timestamps[finite]


def represented_frame_indices(
    point_timestamps_ns,
    frame_timestamps_us,
    boundary_tolerance_ms,
    max_frame_offset_ms,
):
    centers_ns = frame_timestamps_us * 1000
    right = np.searchsorted(centers_ns, point_timestamps_ns)
    right = np.clip(right, 1, len(centers_ns) - 1)
    left = right - 1
    left_distance = np.abs(point_timestamps_ns - centers_ns[left])
    right_distance = np.abs(centers_ns[right] - point_timestamps_ns)
    nearest = np.where(left_distance <= right_distance, left, right)
    nearest_distance = np.minimum(left_distance, right_distance)
    if nearest_distance.max(initial=0) > max_frame_offset_ms * 1e6:
        raise ValueError(
            "Point timestamp is too far from every LiDAR frame: "
            f"{nearest_distance.max() / 1e6:.3f} ms"
        )

    indices = [nearest]
    ambiguous = np.abs(left_distance - right_distance) <= 2 * boundary_tolerance_ms * 1e6
    if np.any(ambiguous):
        indices.extend((left[ambiguous], right[ambiguous]))
    return np.unique(np.concatenate(indices))


def transformed_car_boxes(
    frame_indices,
    lidar_frames,
    bbox_dir,
    T_enu_submap_robot,
    horizontal_padding,
):
    T_submap_robot_enu = np.linalg.inv(T_enu_submap_robot)
    boxes = []
    for frame_idx in frame_indices:
        frame = lidar_frames[int(frame_idx)]
        bbox_path = bbox_dir / f"{frame.frame}.txt"
        if not bbox_path.is_file():
            raise FileNotFoundError(f"Missing bbox file: {bbox_path}")
        frame_boxes = BoundingBoxes()
        frame_boxes.load_from_file(str(bbox_path))
        T_submap_robot_source_lidar = T_submap_robot_enu @ frame.pose
        rotation = T_submap_robot_source_lidar[:3, :3]
        translation = T_submap_robot_source_lidar[:3, 3]
        for box in frame_boxes.bbs:
            if box.label != "Car":
                continue
            center = rotation @ box.pos[:, 0] + translation
            box_rotation = rotation @ box.rot
            half_extent = box.extent[:, 0] / 2
            half_extent[:2] += horizontal_padding
            boxes.append((center, box_rotation, half_extent))
    return boxes


def vertices_inside_boxes(vertices, boxes):
    inside = np.zeros(len(vertices), dtype=bool)
    for center, rotation, half_extent in boxes:
        aabb_half_extent = np.abs(rotation) @ half_extent
        candidates = np.flatnonzero(
            ~inside
            & np.all(np.abs(vertices - center) <= aabb_half_extent, axis=1)
        )
        if len(candidates):
            local = (vertices[candidates] - center) @ rotation
            inside[candidates] = np.all(
                np.abs(local) <= half_extent,
                axis=1,
            )
    return inside


def compact_mesh(vertices, triangles, source_indices, vertex_keep):
    face_keep = np.all(vertex_keep[triangles], axis=1)
    kept_triangles = triangles[face_keep]
    if not len(kept_triangles):
        raise RuntimeError("Filtering removed every mesh triangle.")
    referenced = np.unique(kept_triangles)
    remap = np.full(len(vertices), -1, dtype=np.int32)
    remap[referenced] = np.arange(len(referenced), dtype=np.int32)
    return (
        vertices[referenced],
        remap[kept_triangles],
        source_indices[referenced],
        face_keep,
    )


def self_test():
    angle = np.deg2rad(30)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    )
    vertices = np.array([[0, 0, 0], [3, 0, 0], [0, 3, 0], [3, 3, 0]], dtype=float)
    inside = vertices_inside_boxes(
        vertices,
        [(np.zeros(3), rotation, np.array([1.0, 0.5, 1.0]))],
    )
    assert inside.tolist() == [True, False, False, False]
    compacted = compact_mesh(
        vertices,
        np.array([[0, 1, 2], [1, 2, 3]], dtype=np.int32),
        np.arange(4, dtype=np.int32),
        ~inside,
    )
    assert compacted[0].shape == (3, 3)
    assert compacted[1].tolist() == [[0, 1, 2]]
    assert compacted[2].tolist() == [1, 2, 3]
    print("self-test passed")


def parse_args():
    parser = argparse.ArgumentParser(description="Remove car-box geometry from saved submap meshes.")
    parser.add_argument("--sequence-id")
    parser.add_argument("--submap-stamp", type=int)
    parser.add_argument("--mesh-root", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--bbox-root", type=Path)
    parser.add_argument("--horizontal-padding", type=float, default=0.2)
    parser.add_argument("--boundary-tolerance-ms", type=float, default=1.0)
    parser.add_argument("--max-frame-offset-ms", type=float, default=60.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if not args.sequence_id:
        raise ValueError("--sequence-id is required unless --self-test is used.")
    if args.horizontal_padding < 0 or args.boundary_tolerance_ms < 0:
        raise ValueError("Padding and timestamp tolerance must be nonnegative.")

    root = Path(os.environ["VTRROOT"])
    data_root = os.environ["VTRRDATA"]
    results_root = Path(os.environ.get("VTRRESULT", root / "results"))
    mesh_root = args.mesh_root or root / "postprocessing" / "submap_meshes"
    output_root = args.output_root or root / "postprocessing" / "submap_meshes_no_cars"
    bbox_root = args.bbox_root or root / "car_bboxes"
    sequence_id = args.sequence_id

    sequence = BoreasDataset(data_root, [[sequence_id]]).sequences[0]
    lidar_frames = sequence.lidar_frames
    frame_timestamps_us = np.array(
        [frame.timestamp_micro for frame in lidar_frames], dtype=np.int64
    )
    graph_dir = results_root / "lidar" / sequence_id / sequence_id / "graph"
    graph, submaps = get_submaps(graph_dir)
    T_lidar_robot = np.asarray(build_T_lidar_robot(sequence).matrix())
    lidar_by_stamp = {frame.timestamp_micro: frame for frame in lidar_frames}

    input_sequence_dir = mesh_root / sequence_id
    stamps = sorted(
        int(path.name) for path in input_sequence_dir.iterdir() if path.is_dir()
    )
    if args.submap_stamp is not None:
        if args.submap_stamp not in stamps:
            raise ValueError(f"Mesh submap not found: {args.submap_stamp}")
        stamps = [args.submap_stamp]

    totals = np.zeros(5, dtype=np.int64)
    for stamp in stamps:
        input_dir = input_sequence_dir / str(stamp)
        output_dir = output_root / sequence_id / str(stamp)
        if output_dir.exists() and not args.overwrite and not args.dry_run:
            print(f"skip existing: {output_dir}")
            continue

        vertices = np.load(input_dir / "vertices.npy")
        triangles = np.load(input_dir / "triangles.npy")
        source_indices = np.load(input_dir / "source_point_indices.npy")
        if len(vertices) != len(source_indices):
            raise ValueError(f"Vertex/source index length mismatch in {input_dir}")
        if triangles.ndim != 2 or triangles.shape[1] != 3:
            raise ValueError(f"Invalid triangles in {input_dir}: {triangles.shape}")
        if len(triangles) == 0 or triangles.min() < 0 or triangles.max() >= len(vertices):
            raise ValueError(f"Triangle indices are invalid in {input_dir}")

        submap = submaps.get(stamp)
        submap_lidar_frame = lidar_by_stamp.get(stamp)
        if submap is None or submap_lidar_frame is None:
            raise ValueError(f"No graph submap/LiDAR frame for mesh stamp {stamp}")
        point_timestamps = decode_point_timestamps(submap.get_data("pointmap"))
        if len(source_indices) and source_indices.max() >= len(point_timestamps):
            raise ValueError(f"Source point indices are invalid in {input_dir}")
        frame_indices = represented_frame_indices(
            point_timestamps,
            frame_timestamps_us,
            args.boundary_tolerance_ms,
            args.max_frame_offset_ms,
        )
        T_enu_submap_robot = submap_lidar_frame.pose @ T_lidar_robot
        boxes = transformed_car_boxes(
            frame_indices,
            lidar_frames,
            bbox_root / sequence_id,
            T_enu_submap_robot,
            args.horizontal_padding,
        )
        inside = vertices_inside_boxes(vertices, boxes)
        filtered_vertices, filtered_triangles, filtered_sources, face_keep = compact_mesh(
            vertices,
            triangles,
            source_indices,
            ~inside,
        )
        print(
            f"{stamp}: frames={len(frame_indices)} boxes={len(boxes)} "
            f"vertices={len(vertices)}->{len(filtered_vertices)} "
            f"inside={inside.sum()} faces={len(triangles)}->{len(filtered_triangles)}"
        )
        totals += (
            len(vertices),
            len(filtered_vertices),
            int(inside.sum()),
            len(triangles),
            len(filtered_triangles),
        )
        if args.dry_run:
            continue

        output_dir.mkdir(parents=True, exist_ok=True)
        np.save(output_dir / "vertices.npy", filtered_vertices.astype(np.float32, copy=False))
        np.save(output_dir / "triangles.npy", filtered_triangles.astype(np.int32, copy=False))
        np.save(output_dir / "source_point_indices.npy", filtered_sources.astype(np.int32, copy=False))
        with np.load(input_dir / "metadata.npz") as metadata:
            values = {key: metadata[key] for key in metadata.files}
        values.update(
            car_filter_horizontal_padding=np.asarray(args.horizontal_padding),
            car_filter_bbox_frame_count=np.asarray(len(frame_indices), dtype=np.int32),
            car_filter_box_count=np.asarray(len(boxes), dtype=np.int32),
        )
        np.savez(output_dir / "metadata.npz", **values)

    print(
        f"total vertices {totals[0]}->{totals[1]} (inside boxes {totals[2]}), "
        f"faces {totals[3]}->{totals[4]}"
    )
    del graph


if __name__ == "__main__":
    main()
