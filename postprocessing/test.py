import numpy as np
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses 
import os
import time
import cv2
from pathlib import Path
import matplotlib.pyplot as plt

from sensor_msgs_py.point_cloud2 import read_points
import open3d as o3d
from pylgmath import Transformation
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex
import argparse

from vtr_utils.bag_file_parsing import Rosbag2GraphFactory
from vtr_pose_graph.graph_iterators import TemporalIterator, PriviledgedIterator, SpatialIterator
import vtr_pose_graph.graph_utils as g_utils
from scipy.ndimage import gaussian_filter1d, shift

import torch
import nksr

################## 
# Helper Functions
################## 

def get_submap_vertices(graph_dir):
    factory = Rosbag2GraphFactory(graph_dir)
    test_graph = factory.buildGraph()
    print(f"Graph {test_graph} has {test_graph.number_of_vertices} vertices and {test_graph.number_of_edges} edges")

    # initialize the vertices by iterating the edge transforms
    # initializes T_w_v0 as identity
    g_utils.set_world_frame(test_graph, test_graph.root) # test_graph.root is first vertex
    v_start = test_graph.get_vertex((0,0))

    submap_vertices = []
    curr_submap_vid = None
    for vertex, e in TemporalIterator(v_start):
        map_ptr = vertex.get_data("pointmap_ptr")
        teach_v = test_graph.get_vertex(map_ptr.map_vid)
        
        if curr_submap_vid is not teach_v.id:
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


######################
# Point Cloud Plotting
######################

first = True
paused = False
def toggle(vis):
    global paused
    paused = not paused
    return False

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.register_key_callback(ord(' '), toggle)
# vis.create_window(width=770, height=770)
vis.create_window()
origin = np.array([0, 0, 0])
# origin = np.array([0, 0, 0])
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=origin)
vis.add_geometry(frame)
view_ctl = vis.get_view_control()

pcd = o3d.geometry.PointCloud()
vis.poll_events()
vis.update_renderer()


# Grab the user's home directory
boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")

boreas_data = os.getenv("VTRRDATA") # TODO change to use environment variable
bd = BoreasDataset(boreas_data)

device = torch.device("cuda")
reconstructor = nksr.Reconstructor(device)

radar_start_frame = 100 # 412
radar_end_frame = None
radar_start_ts = None
radar_end_ts = None

# Loop through each frame in order (odometry)
for seq in bd.sequences:
    print(f"SequenceID: {seq.ID}")
    if seq.ID != "boreas-2024-12-03-12-54":
        continue
    if radar_end_frame is None:
        radar_end_frame = len(seq.radar_frames) - 1

    # get radar start and end times
    radar_start_ts = seq.radar_frames[radar_start_frame].frame
    radar_end_ts = seq.radar_frames[radar_end_frame].frame

    # define graph folder
    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")

    # get submap vertices
    test_graph, submap_vertices = get_submap_vertices(graph_dir=graph_dir)

    # Get T_lidar_robot
    T_wheel_robot_np = np.array([ # Transforms points from (X-forward, Y-left) TO (Y-forward, X-right)
        [ 0.0, -1.0,  0.0,  0.0],
        [ 1.0,  0.0,  0.0,  0.0],
        [ 0.0,  0.0,  1.0,  0.0],
        [ 0.0,  0.0,  0.0,  1.0]
    ])
    T_wheel_robot = Transformation(T_ba=T_wheel_robot_np)
    T_applanix_wheel = Transformation(T_ba=seq.calib.T_applanix_wheel)
    T_applanix_lidar = Transformation(T_ba=seq.calib.T_applanix_lidar)
    T_lidar_wheel = T_applanix_lidar.inverse() * T_applanix_wheel
    T_lidar_robot = T_lidar_wheel * T_wheel_robot


    lidar_frame_idx = 0
    submap_vertices_idx = 0
    print("Number of Submaps:", len(submap_vertices))
    radar_frame_idx = radar_start_frame
    while (submap_vertices_idx < len(submap_vertices) - 1 and lidar_frame_idx < len(seq.lidar_frames) and radar_frame_idx < radar_end_frame + 1):
        curr_submap = submap_vertices[submap_vertices_idx]
        next_submap = submap_vertices[submap_vertices_idx + 1]
        radar_frame = seq.radar_frames[radar_frame_idx]
        lidar_frame = seq.lidar_frames[lidar_frame_idx]

        if int(radar_frame.frame) < curr_submap.stamp // 1000: # submap timestamps are in ns (radar/lidar in micro s)
            radar_frame_idx += 1
            continue

        if next_submap.stamp // 1000 < int(radar_frame.frame):
            submap_vertices_idx += 1
            continue

        if int(lidar_frame.frame) != curr_submap.stamp // 1000:
            lidar_frame_idx += 1
            continue
        
        # Get submap in lidar frame
        map_pts = extract_points_from_vertex(curr_submap, msg="pointmap")
        print("-" * 10)
        # print(f"map pts shape: {map_pts.shape}")
        print(radar_frame.frame)
        print("Index:", radar_frame_idx)
        # # print(f"intensities shape: {intensities.shape}, Max: {np.max(intensities)}, Min: {np.min(intensities)}")
        # print("-" * 10)
        map_pts = convert_points_to_frame(map_pts, T_lidar_robot)

        # Get submap in current radar frame (lidar --> ENU --> radar)
        T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
        T_enu_radar = Transformation(T_ba=radar_frame.pose)
        map_pts = convert_points_to_frame(map_pts, T_enu_radar.inverse() * T_enu_lidar)
        print(map_pts.shape)

        # Filter only the points at a given elevation range
        x_points = map_pts[0, :]
        y_points = map_pts[1, :] 
        z_points = map_pts[2, :]

        r_vals = np.sqrt(x_points**2 + y_points**2 + z_points**2)
        azimuth = np.arctan2(y_points, x_points)
        elevation = np.arcsin(z_points / r_vals)

        valid_mask = (
            (np.abs(elevation) <= np.deg2rad(90))
            # (z_points < 1.42)
            # (np.abs(azimuth - np.deg2rad(-110.63574)) <= np.deg2rad(1))
        )

        map_pts = map_pts[:, valid_mask]
        # # intensities = intensities[valid_mask]
        
        # plot point cloud
        pcd.points = o3d.utility.Vector3dVector(map_pts.T)    
        pcd, kept_indices = pcd.remove_radius_outlier(
            nb_points=2,
            radius=0.20,
        )  
        radar_frame.unload_data()

        # Mesh Logic
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.2,
                max_nn=30
            )
        )

        pcd.orient_normals_consistent_tangent_plane(k=5)

        xyz = torch.from_numpy(
            np.asarray(pcd.points)
        ).float().to(device)

        normals = torch.from_numpy(
            np.asarray(pcd.normals)
        ).float().to(device)

        field = reconstructor.reconstruct(
            xyz,
            normal=normals,
            detail_level=0.5,
        )

        if field is None:
            raise RuntimeError("NKSR reconstruction returned no field.")

        nksr_mesh = field.extract_dual_mesh(mise_iter=1, trim=True)

        # Convert NKSR tensors into a standard Open3D mesh.
        mesh = o3d.geometry.TriangleMesh(
            vertices=o3d.utility.Vector3dVector(
                nksr_mesh.v.detach().cpu().numpy()
            ),
            triangles=o3d.utility.Vector3iVector(
                nksr_mesh.f.detach().cpu().numpy()
            ),
        )

        mesh.compute_vertex_normals()
        color_mesh_by_height(mesh, cmap_name="jet")
        pcd.paint_uniform_color([0.15, 0.15, 0.15])

        # For video playing in Open3D
        if first:
            first = False
            paused = True
            # vis.add_geometry(pcd)
            print("hello")
            o3d.visualization.draw_geometries(
                [mesh],
                mesh_show_back_face=True,
            )
            
            # Show one normal per point.
            # o3d.visualization.draw_geometries(
            #     [pcd],
            #     point_show_normal=True,
            # )

            # # Look along the Z-axis (into the page)
            view_ctl.set_front([0, 0, -1]) 
            # # Point the X-axis UP
            # view_ctl.set_up([1, 0, 0])     

            # view_ctl.set_front([-2, 0, -1]) 

            # Keep the Z-axis (or whichever is your vertical) pointing UP
            view_ctl.set_up([1, 0, 0])

            # Center on the origin
            view_ctl.set_lookat(origin)
        else:
            vis.update_geometry(pcd)

            # # Look along the Z-axis (into the page)
            # view_ctl.set_front([0, 0, -1]) 
            # # Point the X-axis UP
            # view_ctl.set_up([1, 0, 0])   

            # view_ctl.set_front([-3, 0, -1]) 
            # view_ctl.set_up([1, 0, 0])
            # Center on the origin
            # view_ctl.set_lookat(origin)
        
        t = time.time()
        while time.time() - t < 0.1 or paused:
            vis.poll_events()
            vis.update_renderer()

        # --- INSERT CAPTURE CODE HERE ---
        # 1. Force the GPU to draw the new geometry and camera angle
        # vis.poll_events()
        # vis.update_renderer()

        # # 2. Save the image
        # image_name = f"bev_lidar_images/{radar_frame.frame}.png"
        # vis.capture_screen_image(image_name, do_render=True)
        # --------------------------------
        
        

        radar_frame_idx += 1

    break
