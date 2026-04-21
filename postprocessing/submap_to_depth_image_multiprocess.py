import numpy as np
import math
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses 
from pyboreas.utils.utils import get_inverse_tf
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
from scipy.interpolate import interp1d
from collections import defaultdict
import concurrent.futures
from multiprocessing import shared_memory

################## 
# Helper Functions
################## 

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def make_chunks(seq, chunk_size):
    return [seq[i:i + chunk_size] for i in range(0, len(seq), chunk_size)]

def interpolate_poses_chunk(args):
    poses, times, query_times_chunk = args
    return interpolate_poses(poses, times, query_times_chunk)

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

def shift_polar_chunk(args):
    polar_chunk, doppler_chunk = args
    out = np.empty_like(polar_chunk)

    # performs doppler shift on chunk
    for i in range(len(polar_chunk)):
        out[i] = shift(
            polar_chunk[i],
            shift=-doppler_chunk[i],   # negative to undistort
            order=3,
            mode='nearest'
        )

    return out

def correct_offsets(radar_frame, radar_frame_idx, seq, executor=None, max_workers=4):
    prev_radar_frame = seq.radar_frames[radar_frame_idx - 1]
    next_radar_frame = seq.radar_frames[radar_frame_idx + 1]
    body_rates = [prev_radar_frame.body_rate, radar_frame.body_rate, next_radar_frame.body_rate]
    times_us = [prev_radar_frame.timestamp_micro, radar_frame.timestamp_micro, next_radar_frame.timestamp_micro]

    azimuth_timestamps = radar_frame.timestamps.flatten()
    f = interp1d(times_us, body_rates, axis=0, kind='quadratic')
    azimuth_body_rates = f(azimuth_timestamps) # (400, 6)

    # radar_offset
    shifted_polar = shift(
        radar_frame.polar,
        shift=(0, seq.calib.radar_offset / radar_frame.resolution),
        order=3,
        mode='nearest'
    )

    # Doppler Distortion
    vx = -azimuth_body_rates[:, 0]
    vy = -azimuth_body_rates[:, 1]
    u = vx * np.cos(radar_frame.azimuths) + vy * np.sin(radar_frame.azimuths)
    beta = seq.calib.radar_doppler_beta
    delta_r_d = beta * u
    chirp_sign = np.where(radar_frame.chirp_type == 0, -1, radar_frame.chirp_type)
    doppler_shift = chirp_sign * delta_r_d / radar_frame.resolution # need to SUBTRACT this to get real range

    num_rows = shifted_polar.shape[0]
    chunk_size = math.ceil(num_rows / max_workers)

    polar_chunks = [shifted_polar[i:i + chunk_size] for i in range(0, num_rows, chunk_size)]
    doppler_chunks = [doppler_shift[i:i + chunk_size] for i in range(0, num_rows, chunk_size)]

    args = list(zip(polar_chunks, doppler_chunks))

    if executor is None:
        with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as ex:
            chunk_results = list(ex.map(shift_polar_chunk, args))
    else:
        chunk_results = list(executor.map(shift_polar_chunk, args))

    return np.vstack(chunk_results)

def cen_filter_2d(polar_image, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5):
    """
    Vectorized Cen filter for an entire radar polar image.
    Expects a 2D numpy array of shape (num_azimuths, num_range_bins).
    
    Args:
        polar_image: 2D array of raw radar power returns.
        sigma_gauss: Controls the width of the Gaussian smoothing.
        z_q: Threshold multiplier for zeroing out noise.
        noise_scale: Multiplier to artificially scale the estimated noise floor.
    """
    # 1. Subtract mean per azimuth
    mean_val = np.mean(polar_image, axis=1, keepdims=True)
    q = polar_image - mean_val
    
    # 2. Apply Gaussian filter across the range bins
    p = gaussian_filter1d(q, sigma=sigma_gauss, axis=1, mode='reflect')
    
    # 3. Estimate TRUE noise sigma per azimuth
    neg_mask = q < 0
    count = np.sum(neg_mask, axis=1, keepdims=True)
    
    q_neg_sq = (q * neg_mask) ** 2
    sum_q_neg_sq = np.sum(q_neg_sq, axis=1, keepdims=True)
    
    sigma_q_sq = np.divide(2.0 * sum_q_neg_sq, count, out=np.zeros_like(count, dtype=float), where=count!=0)
    
    # --- THE MAGIC FACTOR IS APPLIED HERE ---
    sigma_q = noise_scale * np.sqrt(sigma_q_sq)
    
    # Fallback for any rows that had exactly 0 negative values
    sigma_q[count == 0] = 0.034
    
    threshold = z_q * sigma_q 
    
    # 4. Compute adaptive weighting
    eps = 1e-8
    pow_p = (p / (sigma_q + eps)) ** 2
    pow_qp = ((q - p) / (sigma_q + eps)) ** 2
    
    npp = np.exp(-0.5 * pow_p)
    nqp = np.exp(-0.5 * pow_qp)
    
    # Re-weight the signal
    y = q * (1.0 - nqp) + p * (nqp - npp)
    
    # 5. Zero out signal values less than threshold
    y[y <= threshold] = 0.0
    
    return y


def extract_range_image_chunk(shm_name, shape, dtype_str, T_azi_chunk, radar_azi_chunk, fov_deg, res):
    hfov = np.deg2rad(fov_deg)
    vfov = np.deg2rad(fov_deg)
    hfov_half = hfov / 2.0
    vfov_half = vfov / 2.0
    az_res_deg = res
    el_res_deg = res

    existing_shm = shared_memory.SharedMemory(name=shm_name)
    try:
        map_pts_enu = np.ndarray(shape, dtype=np.dtype(dtype_str), buffer=existing_shm.buf)

        depth_imgs = []

        for T_azi, radar_azi in zip(T_azi_chunk, radar_azi_chunk):
            T_azi_transform = Transformation(T_ba=T_azi)
            map_pts = convert_points_to_frame(map_pts_enu, T_azi_transform)

            x = map_pts[0, :]
            y = map_pts[1, :]
            z = map_pts[2, :]

            xy = np.sqrt(x * x + y * y)
            r = np.sqrt(x * x + y * y + z * z)
            az = np.arctan2(y, x)
            el = np.arctan2(z, xy)

            daz = wrap_to_pi(az - radar_azi)
            patch_mask = (np.abs(daz) <= hfov_half) & (np.abs(el) <= vfov_half)

            depth_img = patch_angles_to_depth_image(
                az_local=daz[patch_mask],
                el_local=el[patch_mask],
                r=r[patch_mask],
                hfov=hfov,
                vfov=vfov,
                az_res_deg=az_res_deg,
                el_res_deg=el_res_deg,
                min_range=0.0,
            )
            depth_imgs.append(depth_img)

        return depth_imgs

    finally:
        existing_shm.close()


def save_patches(seq_dir, patches, fov, radar_frame):
    # Input Data Directories
    input_dir = Path(seq_dir + "/input")
    array_save_dir = input_dir / f"patch_arrays_{fov}_deg" # 32 bit array (more precision)
    array_save_dir.mkdir(parents=True, exist_ok=True)
    np.save(array_save_dir / f"{radar_frame}.npy", patches)
    return

def save_labels(seq_dir, folder_name, polar, radar_frame):
    # Polar Label Directories
    labels_dir = Path(seq_dir + f"/{folder_name}")
    labels_dir.mkdir(parents=True, exist_ok=True)
    np.save(labels_dir / f"{radar_frame}.npy", polar)
    return
    

######################
# Point Cloud Plotting
######################

# first = True
# paused = False
# def toggle(vis):
#     global paused
#     paused = not paused
#     return False

# vis = o3d.visualization.VisualizerWithKeyCallback()
# vis.register_key_callback(ord(' '), toggle)
# vis.create_window()
# origin = np.array([0, 0, 0])
# frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=origin)
# vis.add_geometry(frame)
# view_ctl = vis.get_view_control()

# pcd = o3d.geometry.PointCloud()
# vis.poll_events()
# vis.update_renderer()

#######################
# MAIN
#######################

# Grab the user's home directory
boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")

boreas_data = os.getenv("VTRRDATA") # TODO change to use environment variable
bd = BoreasDataset(boreas_data)

radar_start_frame = 65 # 65
radar_end_frame = 200 # 200
radar_start_ts = None
radar_end_ts = None

max_workers = 4
num_azimuths = 400
fov_deg = 2.0
res = 0.1
chunk_size = math.ceil(num_azimuths / max_workers)

with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as executor:
    # Loop through each frame in order (odometry)
    for seq in bd.sequences:
        print(f"SequenceID: {seq.ID}")
        print(f"Number of Radar Frames: {len(seq.radar_frames)}")

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
            t0 = time.perf_counter()
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

            # Get radar frame poses for each azimuth
            radar_frame = seq.get_radar(radar_frame_idx)
            poses = [get_inverse_tf(rad_frame.pose) for rad_frame in seq.radar_frames[radar_frame_idx - 1: radar_frame_idx + 2]] # rad_frame.pose is T_enu_radar but interpolate_poses needs T_radar_enu!
            times = [rad_frame.timestamp_micro for rad_frame in seq.radar_frames[radar_frame_idx - 1: radar_frame_idx + 2]]
            query_times = radar_frame.timestamps.flatten().tolist()
            query_time_chunks = make_chunks(query_times, math.ceil(len(query_times) / max_workers))
            
            args = [(poses, times, chunk) for chunk in query_time_chunks]
            chunk_results = list(executor.map(interpolate_poses_chunk, args))
            azimuth_poses = [pose for chunk in chunk_results for pose in chunk]
            t1 = time.perf_counter()
            
            # Get submap in lidar frame
            map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
            map_pts_lidar = convert_points_to_frame(map_pts_robot, T_lidar_robot)

            # Get submap in current radar frame (lidar --> ENU --> radar)
            T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
            T_enu_radar = Transformation(T_ba=radar_frame.pose)
            map_pts_enu = convert_points_to_frame(map_pts_lidar, T_enu_lidar) # map_pts in enu frame

            # Create shared memory for multiprocessing
            pose_chunks = make_chunks(azimuth_poses, chunk_size)
            azi_chunks = make_chunks(radar_frame.azimuths.flatten(), chunk_size)

            shm = shared_memory.SharedMemory(create=True, size=map_pts_enu.nbytes)
            shared_arr = np.ndarray(map_pts_enu.shape, dtype=map_pts_enu.dtype, buffer=shm.buf)
            shared_arr[:] = map_pts_enu

            try:
                chunk_results = list(executor.map(
                    extract_range_image_chunk,
                    [shm.name] * len(pose_chunks),
                    [map_pts_enu.shape] * len(pose_chunks),
                    [map_pts_enu.dtype.str] * len(pose_chunks),
                    pose_chunks,
                    azi_chunks,
                    [fov_deg] * len(pose_chunks), 
                    [res] * len(pose_chunks)
                ))
            finally:
                shm.close()
                shm.unlink()
            
            depth_patches = [img for chunk in chunk_results for img in chunk]
            patches = np.stack(depth_patches).astype(np.float32)

            # for depth_img in depth_patches:
            #     ##################
            #     # Plot Depth Image
            #     ##################

            #     plt.figure(figsize=(6, 6))
            #     plt.imshow(depth_img, origin='upper')
            #     plt.colorbar(label='Range / depth')
            #     plt.title("Depth image")
            #     plt.xlabel("Azimuth bin")
            #     plt.ylabel("Elevation bin")
            #     plt.show()

                # For video playing in Open3D
                # if first:
                #     first = False
                #     paused = True
                #     vis.add_geometry(pcd)
                #     # Look along the Z-axis (into the page)
                #     # view_ctl.set_front([0, 0, -1]) 
                #     # # Point the X-axis UP
                #     # view_ctl.set_up([1, 0, 0])   

                #     view_ctl.set_front([-1, 0, 0])
                #     view_ctl.set_up([0, 0, -1])  
                #     # Center on the origin
                #     view_ctl.set_lookat(origin)
                # else:
                #     vis.update_geometry(pcd)
                #     # Look along the Z-axis (into the page)
                #     # view_ctl.set_front([0, 0, -1]) 
                #     # # Point the X-axis UP
                #     # view_ctl.set_up([1, 0, 0])   

                #     view_ctl.set_front([-1, 0, -1])
                #     view_ctl.set_up([0, 0, -1]) 
                #     # Center on the origin
                #     view_ctl.set_lookat(origin)
                # t = time.time()
                # while time.time() - t < 0.1 or paused:
                #     vis.poll_events()
                #     vis.update_renderer()

            t2 = time.perf_counter()
            shifted_polar = correct_offsets(
                radar_frame, 
                radar_frame_idx, 
                seq, 
                executor=executor, 
                max_workers=max_workers)
            
            filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)            

            t3 = time.perf_counter()

            save_patches(seq_dir=seq.seq_root, patches=patches, fov=fov_deg, radar_frame=radar_frame.frame)
            # save_labels(seq_dir=seq.seq_root, folder_name="labels", polar=shifted_polar, radar_frame=radar_frame.frame)
            # save_labels(seq_dir=seq.seq_root, folder_name="filtered_labels", polar=filtered_polar, radar_frame=radar_frame.frame)


            t4 = time.perf_counter()
            print(f"Elapsed time: {t4 - t0:.6f} s | Pose Interpolation: {t1 - t0:.6f} s | Extract Depth Patches: {t2 - t1:.6f} s | Polar Corrections: {t3 - t2:.6f} s | Saving: {t4 - t3:.6f} s ")

            map_pts = convert_points_to_frame(map_pts_enu, T_enu_radar)
            print("-" * 10)
            print(f"map pts shape: {map_pts.shape}")
            print("-" * 10)

            # # plot point cloud patch
            # pcd.points = o3d.utility.Vector3dVector(map_pts.T)
            # pcd.paint_uniform_color((0.1*curr_submap.run, 0.25*curr_submap.run, 0.45))
            # colors = np.asarray(pcd.colors)

            # # For video playing in Open3D
            # if first:
            #     first = False
            #     paused = True
            #     vis.add_geometry(pcd)
            # else:
            #     vis.update_geometry(pcd)
            # t = time.time()
            # while time.time() - t < 0.1 or paused:
            #     vis.poll_events()
            #     vis.update_renderer()

            radar_frame.unload_data()
            print("radar frame unloaded!")
            radar_frame_idx += 1

        break