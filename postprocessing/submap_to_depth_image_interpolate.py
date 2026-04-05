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
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex, extract_points_from_vertex
import argparse

from vtr_utils.bag_file_parsing import Rosbag2GraphFactory
from vtr_pose_graph.graph_iterators import TemporalIterator, PriviledgedIterator, SpatialIterator
import vtr_pose_graph.graph_utils as g_utils
from scipy.ndimage import gaussian_filter1d, shift
from collections import defaultdict

################## 
# Helper Functions
################## 

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def patch_angles_to_depth_image(
    az_local,
    el_local,
    r,
    hfov,
    vfov,
    az_res_deg=0.1,
    el_res_deg=0.1,
    min_range=0.0,
):
    az_res = np.deg2rad(az_res_deg)
    el_res = np.deg2rad(el_res_deg)

    # Force odd image dimensions so there is an exact center pixel
    W = int(np.round(hfov / az_res)) + 1
    H = int(np.round(vfov / el_res)) + 1

    assert W % 2 == 1
    assert H % 2 == 1

    u_c = (W - 1) / 2.0
    v_c = (H - 1) / 2.0

    depth_img = np.full((H, W), np.inf, dtype=np.float32)

    hfov_half = hfov / 2.0
    vfov_half = vfov / 2.0

    valid = (
        np.isfinite(r)
        & np.isfinite(az_local)
        & np.isfinite(el_local)
        & (r > min_range)
        & (az_local >= -hfov_half)
        & (az_local <= hfov_half)
        & (el_local >= -vfov_half)
        & (el_local <= vfov_half)
    )

    if not np.any(valid):
        depth_img[~np.isfinite(depth_img)] = 0
        return depth_img

    az_local = az_local[valid].astype(np.float32)
    el_local = el_local[valid].astype(np.float32)
    r = r[valid].astype(np.float32)

    # Continuous pixel-center coordinates
    u_f = az_local / az_res + u_c
    v_f = -el_local / el_res + v_c

    # Hard assignment: nearest pixel center wins
    u = np.round(u_f).astype(np.int32)
    v = np.round(v_f).astype(np.int32)

    in_bounds = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if not np.any(in_bounds):
        depth_img[~np.isfinite(depth_img)] = 0
        return depth_img

    u = u[in_bounds]
    v = v[in_bounds]
    r = r[in_bounds]

    np.minimum.at(depth_img, (v, u), r)
    depth_img[~np.isfinite(depth_img)] = 0
    return depth_img


def angular_patch_to_depth_image_bilinear(
    patch_r,
    patch_az_local,
    patch_el,
    hfov,
    vfov,
    az_res_deg=0.1,
    el_res_deg=0.1,
    min_range=0.0,
):
    az_res = np.deg2rad(az_res_deg)
    el_res = np.deg2rad(el_res_deg)

    # Force odd image dimensions so there is an exact center pixel
    W = int(np.round(hfov / az_res)) + 1
    H = int(np.round(vfov / el_res)) + 1

    assert W % 2 == 1
    assert H % 2 == 1

    u_c = (W - 1) / 2.0
    v_c = (H - 1) / 2.0

    depth_img = np.full((H, W), np.inf, dtype=np.float32)

    if patch_r.size == 0:
        depth_img[~np.isfinite(depth_img)] = 0
        return depth_img, {
            "W": W,
            "H": H,
            "u_c": u_c,
            "v_c": v_c,
            "az_local": np.empty((0,), dtype=np.float32),
            "el_local": np.empty((0,), dtype=np.float32),
            "r": np.empty((0,), dtype=np.float32),
            "u_f": np.empty((0,), dtype=np.float32),
            "v_f": np.empty((0,), dtype=np.float32),
            "weight_sum": np.zeros((H, W), dtype=np.float32),
            "depth_sum": np.zeros((H, W), dtype=np.float32),
        }

    hfov_half = hfov / 2.0
    vfov_half = vfov / 2.0

    valid = (
        np.isfinite(patch_r)
        & np.isfinite(patch_az_local)
        & np.isfinite(patch_el)
        & (patch_r > min_range)
        & (patch_az_local >= -hfov_half)
        & (patch_az_local <= hfov_half)
        & (patch_el >= -vfov_half)
        & (patch_el <= vfov_half)
    )

    if not np.any(valid):
        depth_img[~np.isfinite(depth_img)] = 0
        return depth_img, {
            "W": W,
            "H": H,
            "u_c": u_c,
            "v_c": v_c,
            "az_local": np.empty((0,), dtype=np.float32),
            "el_local": np.empty((0,), dtype=np.float32),
            "r": np.empty((0,), dtype=np.float32),
            "u_f": np.empty((0,), dtype=np.float32),
            "v_f": np.empty((0,), dtype=np.float32),
            "weight_sum": np.zeros((H, W), dtype=np.float32),
            "depth_sum": np.zeros((H, W), dtype=np.float32),
        }

    az_local = patch_az_local[valid].astype(np.float32)
    el_local = patch_el[valid].astype(np.float32)
    r_local = patch_r[valid].astype(np.float32)

    # Continuous pixel-center coordinates
    # Integer pixel indices correspond to pixel centers
    u_f = az_local / az_res + u_c
    v_f = -el_local / el_res + v_c

    # Keep only points whose bilinear neighborhood can touch the image
    valid2 = (
        (u_f >= -1.0) & (u_f <= W) &
        (v_f >= -1.0) & (v_f <= H)
    )

    if not np.any(valid2):
        depth_img[~np.isfinite(depth_img)] = 0
        return depth_img, {
            "W": W,
            "H": H,
            "u_c": u_c,
            "v_c": v_c,
            "az_local": np.empty((0,), dtype=np.float32),
            "el_local": np.empty((0,), dtype=np.float32),
            "r": np.empty((0,), dtype=np.float32),
            "u_f": np.empty((0,), dtype=np.float32),
            "v_f": np.empty((0,), dtype=np.float32),
            "weight_sum": np.zeros((H, W), dtype=np.float32),
            "depth_sum": np.zeros((H, W), dtype=np.float32),
        }

    az_local = az_local[valid2]
    el_local = el_local[valid2]
    r_local = r_local[valid2]
    u_f = u_f[valid2]
    v_f = v_f[valid2]

    # Neighboring pixel centers
    u0 = np.floor(u_f).astype(np.int32)
    v0 = np.floor(v_f).astype(np.int32)
    u1 = u0 + 1
    v1 = v0 + 1

    du = u_f - u0
    dv = v_f - v0

    w00 = (1.0 - du) * (1.0 - dv)  # top-left
    w01 = du * (1.0 - dv)          # top-right
    w10 = (1.0 - du) * dv          # bottom-left
    w11 = du * dv                  # bottom-right

    depth_sum = np.zeros((H, W), dtype=np.float32)
    weight_sum = np.zeros((H, W), dtype=np.float32)

    def splat(ui, vi, w):
        m = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H) & (w > 0)
        if np.any(m):
            np.add.at(depth_sum, (vi[m], ui[m]), w[m] * r_local[m])
            np.add.at(weight_sum, (vi[m], ui[m]), w[m])

    splat(u0, v0, w00)
    splat(u1, v0, w01)
    splat(u0, v1, w10)
    splat(u1, v1, w11)

    valid_pix = weight_sum > 0.0
    depth_img[valid_pix] = depth_sum[valid_pix] / weight_sum[valid_pix]
    depth_img[~np.isfinite(depth_img)] = 0

    return depth_img, {
        "W": W,
        "H": H,
        "u_c": u_c,
        "v_c": v_c,
        "az_local": az_local,
        "el_local": el_local,
        "r": r_local,
        "u_f": u_f,
        "v_f": v_f,
        "weight_sum": weight_sum,
        "depth_sum": depth_sum,
    }


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
        
        if curr_submap_vid != teach_v.id:
            submap_vertices.append(teach_v)
            curr_submap_vid = teach_v.id

    return test_graph, submap_vertices


def correct_offsets(radar_frame, seq):
    # radar_offset
    shifted_polar = shift(radar_frame.polar, shift=(0, seq.calib.radar_offset / radar_frame.resolution), order=3, mode='nearest')

    # Doppler Distortion
    vx = -radar_frame.body_rate[0]
    vy = -radar_frame.body_rate[1]
    u = vx * np.cos(radar_frame.azimuths) + vy * np.sin(radar_frame.azimuths)
    beta = seq.calib.radar_doppler_beta
    delta_r_d = beta * u
    chirp_sign = np.where(radar_frame.chirp_type == 0, -1, radar_frame.chirp_type)
    doppler_shift = chirp_sign * delta_r_d / radar_frame.resolution # need to SUBTRACT this to get real range

    for idx in range(400):
        shifted_polar[idx] = shift(
            shifted_polar[idx],
            shift=-doppler_shift[idx],   # negative to undistort
            order=3,
            mode='nearest'
        )

    return shifted_polar


def save_patches_and_labels(save_dir, patches, polar, radar_frame):
    # Input Data Directories
    input_dir = Path(save_dir + "/input")
    array_save_dir = input_dir / "patch_arrays_5_deg_no_interp" # 32 bit array (more precision)
    
    # Polar Label Directories
    labels_dir = Path(save_dir + "/shifted_labels")

    array_save_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)
    np.save(array_save_dir / f"{radar_frame}.npy", patches)
    np.save(labels_dir / f"{radar_frame}.npy", polar)
    return

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
vis.create_window()
origin = np.array([0, 0, 0])
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=origin)
vis.add_geometry(frame)
view_ctl = vis.get_view_control()

pcd = o3d.geometry.PointCloud()
vis.poll_events()
vis.update_renderer()

#######################
# MAIN
#######################

# Grab the user's home directory
boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")

boreas_data = os.getenv("VTRRDATA") # TODO change to use environment variable
bd = BoreasDataset(boreas_data)

radar_start_frame = 65
radar_end_frame = 200
radar_start_ts = None
radar_end_ts = None

# Loop through each frame in order (odometry)
for seq in bd.sequences:
    print(f"SequenceID: {seq.ID}")

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
    radar_frame_idx = radar_start_frame
    while (submap_vertices_idx < len(submap_vertices) - 1 and lidar_frame_idx < len(seq.lidar_frames) and radar_frame_idx < radar_end_frame + 1):
        curr_submap = submap_vertices[submap_vertices_idx]
        next_submap = submap_vertices[submap_vertices_idx + 1]
        prev_radar_frame = seq.radar_frames[radar_frame_idx - 1]
        radar_frame = seq.radar_frames[radar_frame_idx]
        next_radar_frame = seq.radar_frames[radar_frame_idx + 1]
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

        # Get radar frame poses for each azimuth
        radar_frame = seq.get_radar(radar_frame_idx)
        poses = [prev_radar_frame.pose, radar_frame.pose, next_radar_frame.pose] # poses are T_enu_radar
        times = [prev_radar_frame.timestamp_micro, radar_frame.timestamp_micro, next_radar_frame.timestamp_micro]
        query_times = radar_frame.timestamps.flatten().tolist()
        azimuth_poses = interpolate_poses(poses, times, query_times)
        azimuth_transform_poses = [Transformation(T_ba=pose) for pose in azimuth_poses]
        
        # Get submap in lidar frame
        map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
        map_pts_lidar = convert_points_to_frame(map_pts_robot, T_lidar_robot)

        # Get submap in current radar frame (lidar --> ENU --> radar)
        T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
        T_enu_radar = Transformation(T_ba=radar_frame.pose)
        map_pts_enu = convert_points_to_frame(map_pts_lidar, T_enu_lidar) # map_pts in enu frame

        t0 = time.perf_counter()
        # naively get map points in each azimuth pose
        map_pts_all = [convert_points_to_frame(map_pts_enu, T_azi.inverse()) for T_azi in azimuth_transform_poses]

        # convert to spherical
        pts = np.stack(map_pts_all, axis=0) # (400, 3, N)
        x = pts[:, 0, :]
        y = pts[:, 1, :]
        z = pts[:, 2, :]

        xy = np.sqrt(x*x + y*y)
        r  = np.sqrt(x*x + y*y + z*z)
        az = np.arctan2(y, x)
        el = np.arctan2(z, xy)

        # extract small patch of each point cloud
        hfov = np.deg2rad(5.0)
        vfov = np.deg2rad(5.0)
        hfov_half = hfov / 2.0
        vfov_half = vfov / 2.0
        daz = wrap_to_pi(az - radar_frame.azimuths) # radar_frame.azimuths shape (400, 1); ranges from [-pi, pi]
        patch_mask = (np.abs(daz) <= hfov_half) & (np.abs(el) <= vfov_half)
        patch_pts_all = [pts[i, :, patch_mask[i]] for i in range(pts.shape[0])] # all patches as a list
        
        patch_r_all        = [r[i,   patch_mask[i]] for i in range(r.shape[0])]
        patch_az_local_all = [daz[i, patch_mask[i]] for i in range(daz.shape[0])]
        patch_el_all       = [el[i,  patch_mask[i]] for i in range(el.shape[0])]

        az_res_deg = 0.1
        el_res_deg = 0.1

        depth_patches = []
        
        for i in range(len(patch_r_all)):
            # depth_img, meta = angular_patch_to_depth_image_bilinear(
            #     patch_r=patch_r_all[i],
            #     patch_az_local=patch_az_local_all[i],
            #     patch_el=patch_el_all[i],
            #     hfov=hfov,
            #     vfov=vfov,
            #     az_res_deg=az_res_deg,
            #     el_res_deg=el_res_deg,
            #     min_range=0.0,
            # )

            depth_img = patch_angles_to_depth_image(
                az_local=patch_az_local_all[i],
                el_local=patch_el_all[i],
                r=patch_r_all[i],
                hfov=hfov,
                vfov=vfov,
                az_res_deg=az_res_deg,
                el_res_deg=el_res_deg,
                min_range=0.0,
            )
            depth_patches.append(depth_img)

            ###################
            # Patch Point Cloud
            ###################

            # # map_pts = patch_pts_all[i]
            # map_pts = map_pts_all[i]
            # # Filter only the points at a given elevation range
            # x_points = map_pts[0, :]
            # y_points = map_pts[1, :] 
            # z_points = map_pts[2, :]

            # r_vals = np.sqrt(x_points**2 + y_points**2 + z_points**2)
            # azimuth = np.arctan2(y_points, x_points)
            # elevation = np.arcsin(z_points / r_vals)

            # valid_mask = (
            #     (np.abs(elevation) <= np.deg2rad(2)) &
            #     (np.abs(azimuth - np.deg2rad(-110.63574)) <= np.deg2rad(2))
            # )

            # if map_pts.shape[0] == 0 or i != 277:
            #     print(f"Skipping azimuth {i}")
            #     continue

            # # plot point cloud patch
            # map_pts = map_pts[:, valid_mask]
            # print(np.min(r_vals[valid_mask]))
            # print(r_vals[valid_mask])
            # pcd.points = o3d.utility.Vector3dVector(map_pts.T)

            # z = map_pts[2, :]  # height
            # z_min = np.min(z)
            # z_max = np.max(z)

            # # avoid divide-by-zero if all points have same height
            # if z_max > z_min:
            #     z_norm = (z - z_min) / (z_max - z_min)
            # else:
            #     z_norm = np.zeros_like(z)

            # # use matplotlib colormap
            # cmap = plt.get_cmap("turbo")   # or "viridis", "jet"
            # colors = cmap(z_norm)[:, :3]   # drop alpha channel
            # pcd.colors = o3d.utility.Vector3dVector(colors)


            ##################
            # Plot Depth Image
            ##################

            # plt.figure(figsize=(6, 6))
            # plt.imshow(depth_img, origin='upper')
            # plt.colorbar(label='Range / depth')
            # plt.title("Depth image")
            # plt.xlabel("Azimuth bin")
            # plt.ylabel("Elevation bin")
            # plt.show()

            # For video playing in Open3D
            # if first:
            #     first = False
            #     paused = True
            #     vis.add_geometry(pcd)
            #     # Look along the Z-axis (into the page)
            #     view_ctl.set_front([0, 0, -1]) 
            #     # Point the X-axis UP
            #     view_ctl.set_up([1, 0, 0])     
            #     # Center on the origin
            #     view_ctl.set_lookat(origin)
            # else:
            #     vis.update_geometry(pcd)
            #     # Look along the Z-axis (into the page)
            #     view_ctl.set_front([0, 0, -1]) 
            #     # Point the X-axis UP
            #     view_ctl.set_up([1, 0, 0])     
            #     # Center on the origin
            #     view_ctl.set_lookat(origin)
            # t = time.time()
            # while time.time() - t < 0.1 or paused:
            #     vis.poll_events()
            #     vis.update_renderer()

        patches = np.stack(depth_patches).astype(np.float32)
        shifted_polar = correct_offsets(radar_frame, seq)

        save_patches_and_labels(seq.seq_root, patches, shifted_polar, radar_frame.frame)

        t1 = time.perf_counter()
        print(f"Elapsed time: {t1 - t0:.6f} s")

        map_pts = map_pts_all[199]

        # plot point cloud patch
        pcd.points = o3d.utility.Vector3dVector(map_pts.T)
        pcd.paint_uniform_color((0.1*curr_submap.run, 0.25*curr_submap.run, 0.45))
        colors = np.asarray(pcd.colors)

        # For video playing in Open3D
        if first:
            first = False
            paused = True
            vis.add_geometry(pcd)
        else:
            vis.update_geometry(pcd)
        t = time.time()
        while time.time() - t < 0.1 or paused:
            vis.poll_events()
            vis.update_renderer()

        radar_frame.unload_data()
        print("radar frame unloaded!")
        radar_frame_idx += 1
    break