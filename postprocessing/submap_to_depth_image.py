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

def correct_offsets(radar_frame, seq):
    # radar_offset
    shifted_polar = shift(radar_frame.polar, shift=(0, seq.calib.radar_offset / radar_frame.resolution), order=3, mode='nearest')

    # Doppler Distortion
    vx = radar_frame.body_rate[0]
    vy = radar_frame.body_rate[1]
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


def get_depth_image(sequence_dir, map_pts, depth_image, radar_timestamp, save=True):
    data_dir = Path(sequence_dir)
    img_save_dir = data_dir / "depth_images" 
    flipped_img_save_dir = data_dir / "flipped_depth_images" 
    array_save_dir = data_dir / "depth_arrays" # 32 bit array (more precision)

    img_save_dir.mkdir(parents=True, exist_ok=True)
    flipped_img_save_dir.mkdir(parents=True, exist_ok=True)
    array_save_dir.mkdir(parents=True, exist_ok=True)
    
    x_points = map_pts[0, :]
    y_points = map_pts[1, :] 
    z_points = map_pts[2, :]

    r_vals = np.sqrt(x_points**2 + y_points**2 + z_points**2)
    azimuth = np.arctan2(y_points, x_points)
    elevation = np.arcsin(z_points / r_vals)

    # Check that it's within bounds
    valid_mask = (
        (elevation >= depth_image.elevation_lower) & (elevation < depth_image.elevation_upper) &
        (azimuth >= depth_image.azimuth_lower) & (azimuth < depth_image.azimuth_upper) &
        (r_vals >= depth_image.min_range) & (r_vals <= depth_image.max_range)
    )

    # Filter the arrays using the mask
    r_valid = r_vals[valid_mask]
    azi_valid = azimuth[valid_mask]
    ele_valid = elevation[valid_mask]

    # Calculate pixel coordinates for all valid points
    u = ((azi_valid - depth_image.azimuth_lower) / depth_image.azimuth_res).astype(np.int32)
    v = ((depth_image.elevation_upper - ele_valid) / depth_image.elevation_res).astype(np.int32)

    # initialize with closest value in each pixel
    depth_image.image.fill(np.inf) 
    np.minimum.at(depth_image.image, (v, u), r_valid)
    depth_image.image[depth_image.image == np.inf] = 0.0

    if save:
        # Save raw linear depth (32-bit)
        np.save(array_save_dir / f"{radar_timestamp}.npy", depth_image.image)

        # Save RGB colour depth map
        vis_img_8bit = np.zeros_like(depth_image.image, dtype=np.uint8)
        valid_pixels = depth_image.image > 0
        linear_pct = 2 * (depth_image.image[valid_pixels] - depth_image.min_range) / (depth_image.max_range - depth_image.min_range)
        scaled_depth = 255.0 * (1.0 - linear_pct)
        vis_img_8bit[valid_pixels] = np.clip(scaled_depth, 0, 255).astype(np.uint8)

        # Apply the Turbo colormap (Creates the RGB image)
        color_depth = cv2.applyColorMap(vis_img_8bit, cv2.COLORMAP_TURBO)
        
        # Set the empty background pixels back to pure black
        color_depth[vis_img_8bit == 0] = [0, 0, 0]

        cv2.imwrite(str(img_save_dir / f"{radar_timestamp}.png"), color_depth)
        
        # Since radar is upside down, also save flipped image
        color_depth_flipped = cv2.flip(color_depth, 0)
        cv2.imwrite(str(flipped_img_save_dir / f"{radar_timestamp}.png"), color_depth_flipped)


def extract_patches_for_azimuths(depth_image, azimuths_deg, target_elevation_deg=0.0):
    H, W = depth_image.image.shape
    hfov_deg = depth_image.hfov_deg
    vfov_deg = depth_image.vfov_deg
    
    patch_width = int(round(hfov_deg / depth_image.azimuth_res_deg))
    patch_height = int(round(vfov_deg / depth_image.elevation_res_deg))
    
    # Find vertical center
    v_center = int(round((depth_image.elevation_upper_deg - target_elevation_deg) / depth_image.elevation_res_deg))
    
    # Calculate valid image bounds for vertical axis
    v_start_ideal = v_center - patch_height // 2
    v_end_ideal = v_start_ideal + patch_height
    
    v_start_valid = max(0, v_start_ideal)
    v_end_valid = min(H, v_end_ideal)
    
    # Determine where the valid data belongs inside our zero-padded patch
    p_start = v_start_valid - v_start_ideal
    p_end = p_start + (v_end_valid - v_start_valid)
    
    patches = []
    
    for az_deg in azimuths_deg:
        # 4. Flawless Horizontal Center (Wrap-safe)
        # Shift so lower bound is 0, wrap, then divide by resolution
        az_shifted = (az_deg - depth_image.azimuth_lower_deg) % (depth_image.azimuth_upper_deg - depth_image.azimuth_lower_deg)
        u_center = int(round(az_shifted / depth_image.azimuth_res_deg))
        
        # 5. Generate wrap-around column indices
        u_start = u_center - patch_width // 2
        u_end = u_start + patch_width
        u_indices = np.arange(u_start, u_end) % W
        
        # 6. Initialize an empty (zero-padded) patch
        patch = np.zeros((patch_height, patch_width), dtype=depth_image.image.dtype)
        
        # 7. Safely inject the valid rows into the empty patch
        if v_end_valid > v_start_valid:
            # We slice the valid rows, then use fancy indexing for the wrapped columns
            patch[p_start:p_end, :] = depth_image.image[v_start_valid:v_end_valid, u_indices]
            
        patches.append(patch)
        
    return np.stack(patches)


def save_patches_and_labels(save_dir, patches, polar, depth_image, radar_frame, save_batch=True):
    # Input Data Directories
    input_dir = Path(save_dir + "/input")
    img_save_dir = input_dir / "patch_images" 
    flipped_img_save_dir = input_dir / "flipped_patch_images" 
    array_save_dir = input_dir / "patch_arrays" # 32 bit array (more precision)
    
    # Polar Label Directories
    labels_dir = Path(save_dir + "/shifted_labels")
    labels_save_dir = labels_dir / radar_frame

    if save_batch:
        array_save_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)
        np.save(array_save_dir / f"{radar_frame}.npy", patches)
        np.save(labels_dir / f"{radar_frame}.npy", polar)
        return

    img_save_dir.mkdir(parents=True, exist_ok=True)
    flipped_img_save_dir.mkdir(parents=True, exist_ok=True)
    array_save_dir.mkdir(parents=True, exist_ok=True)
    labels_save_dir.mkdir(parents=True, exist_ok=True)

    # Loop through all patches
    for i, patch in enumerate(patches):
        patch_name = f"{radar_frame}_patch_{i}"
        np.save(array_save_dir / f"{patch_name}.npy", patch)

        # Save RGB colour depth map
        vis_img_8bit = np.zeros_like(patch, dtype=np.uint8)
        valid_pixels = patch > 0
        linear_pct = 2 * (patch[valid_pixels] - depth_image.min_range) / (depth_image.max_range - depth_image.min_range)
        scaled_depth = 255.0 * (1.0 - linear_pct)
        vis_img_8bit[valid_pixels] = np.clip(scaled_depth, 0, 255).astype(np.uint8)

        # Apply the Turbo colormap (Creates the RGB image)
        color_depth = cv2.applyColorMap(vis_img_8bit, cv2.COLORMAP_TURBO)
        
        # Set the empty background pixels back to pure black
        color_depth[vis_img_8bit == 0] = [0, 0, 0]

        cv2.imwrite(str(img_save_dir / f"{patch_name}.png"), color_depth)
        
        # Since radar is upside down, also save flipped image
        color_depth_flipped = cv2.flip(color_depth, 0)
        cv2.imwrite(str(flipped_img_save_dir / f"{patch_name}.png"), color_depth_flipped)

        # Save Labels + Waveform images
        np.save(labels_save_dir / f"{patch_name}.npy", polar[i])

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
origin = np.array([0, 0, 2.42])
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0, origin=origin)
vis.add_geometry(frame)
view_ctl = vis.get_view_control()

pcd = o3d.geometry.PointCloud()
vis.poll_events()
vis.update_renderer()

########################
# Depth Image Generation
########################

class DepthImage:
    def __init__(
        self, 
        elevation_res_deg=0.1, 
        azimuth_res_deg=0.1, 
        elevation_lower_deg=-15.0, 
        elevation_upper_deg=15.0, 
        azimuth_lower_deg=-180.0, 
        azimuth_upper_deg=180.0, 
        hfov_deg=5.0,
        vfov_deg=5.0,
        max_range=200.0,
        min_range=1.0
    ):
        # Degree parameters
        self.elevation_res_deg = elevation_res_deg
        self.azimuth_res_deg = azimuth_res_deg
        
        self.elevation_lower_deg = elevation_lower_deg
        self.elevation_upper_deg = elevation_upper_deg
        self.azimuth_lower_deg = azimuth_lower_deg
        self.azimuth_upper_deg = azimuth_upper_deg

        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg
        
        # Convert and store bounds in radians
        self.elevation_res = np.deg2rad(elevation_res_deg)
        self.azimuth_res = np.deg2rad(azimuth_res_deg)
        
        self.elevation_lower = np.deg2rad(elevation_lower_deg)
        self.elevation_upper = np.deg2rad(elevation_upper_deg)
        self.azimuth_lower = np.deg2rad(azimuth_lower_deg)
        self.azimuth_upper = np.deg2rad(azimuth_upper_deg)
        
        # Calculate image dimensions (using round to prevent floating point truncation errors)
        self.H = int(np.round((self.elevation_upper - self.elevation_lower) / self.elevation_res))
        self.W = int(np.round((self.azimuth_upper - self.azimuth_lower) / self.azimuth_res))
        
        self.max_range = max_range
        self.min_range = min_range
        
        # Initialize the actual image array 
        # (Filled with zeros. For depth maps, you might also consider np.inf)
        self.image = np.zeros((self.H, self.W), dtype=np.float32)

    def __repr__(self):
        """Provides a clean string representation when you print() the object."""
        return (f"<DepthImage | Size: {self.W}x{self.H} | "
                f"Elev: {np.rad2deg(self.elevation_lower):.1f}° to {np.rad2deg(self.elevation_upper):.1f}° | "
                f"Azim: {np.rad2deg(self.azimuth_lower):.1f}° to {np.rad2deg(self.azimuth_upper):.1f}°>")



# Grab the user's home directory
boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")

boreas_data = os.getenv("VTRRDATA") # TODO change to use environment variable
bd = BoreasDataset(boreas_data)

radar_start_frame = 65
radar_end_frame = 1000
radar_start_ts = None
radar_end_ts = None

# Loop through each frame in order (odometry)
for seq in bd.sequences:
    seq = bd.sequences[1]
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
        map_pts, intensities = extract_points_from_vertex(curr_submap, msg="pointmap")
        print("-" * 10)
        print(f"map pts shape: {map_pts.shape}")
        print(f"intensities shape: {intensities.shape}, Max: {np.max(intensities)}, Min: {np.min(intensities)}")
        print("-" * 10)
        map_pts = convert_points_to_frame(map_pts, T_lidar_robot)

        # Get submap in current radar frame (lidar --> ENU --> radar)
        T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
        T_enu_radar = Transformation(T_ba=radar_frame.pose)
        map_pts = convert_points_to_frame(map_pts, T_enu_radar.inverse() * T_enu_lidar)

        # Filter only the points at a given elevation range
        x_points = map_pts[0, :]
        y_points = map_pts[1, :] 
        z_points = map_pts[2, :]

        r_vals = np.sqrt(x_points**2 + y_points**2 + z_points**2)
        azimuth = np.arctan2(y_points, x_points)
        elevation = np.arcsin(z_points / r_vals)

        valid_mask = (
            (np.abs(elevation) <= np.deg2rad(5)) 
            # (np.abs(azimuth - np.deg2rad(-110.63574)) <= np.deg2rad(1))
        )

        map_pts = map_pts[:, valid_mask]
        intensities = intensities[valid_mask]
        
        # plot point cloud
        pcd.points = o3d.utility.Vector3dVector(map_pts.T)

        i_min = np.min(intensities)
        i_max = np.max(intensities)

        # avoid divide-by-zero if all points have same height
        if i_max > i_min:
            i_norm = (intensities - i_min) / (i_max - i_min)
            # i_norm = np.power(i_norm, 2.0)
        else:
            i_norm = np.zeros_like(intensities)

        # use matplotlib colormap
        cmap = plt.get_cmap("turbo")   # or "viridis", "jet"
        colors = cmap(i_norm)[:, :3]   # drop alpha channel
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Save depth images
        depth_image = DepthImage()
        get_depth_image(seq.seq_root + "/input", map_pts, depth_image, radar_frame.frame, save=False)

        # # Extract depth image patches using radar azimuths
        # radar_frame = seq.get_radar(radar_frame_idx)
        # azimuths_deg = np.rad2deg(radar_frame.azimuths.flatten())
        # patches = extract_patches_for_azimuths(depth_image, azimuths_deg)
        # print(f"Extracted shape: {patches.shape} -> (N_patches, Height, Width)")

        # # Process polar image
        # shifted_polar = correct_offsets(radar_frame, seq)
        # # filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)
        
        # # Save patches and labels
        # save_patches_and_labels(seq.seq_root, patches, shifted_polar, depth_image, radar_frame.frame, save_batch=True)
        radar_frame.unload_data()

        # For video playing in Open3D
        if first:
            first = False
            paused = True
            vis.add_geometry(pcd)

            # # Look along the Z-axis (into the page)
            view_ctl.set_front([0, 0, -1]) 
            # # Point the X-axis UP
            # view_ctl.set_up([1, 0, 0])     

            # view_ctl.set_front([-3, 0, -1]) 

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
        
        # --- INSERT CAPTURE CODE HERE ---
        # 1. Force the GPU to draw the new geometry and camera angle
        # vis.poll_events()
        # vis.update_renderer()

        # # 2. Save the image
        # image_name = f"angled_lidar_images/{radar_frame.frame}.png"
        # vis.capture_screen_image(image_name, do_render=True)
        # --------------------------------
        
        t = time.time()
        while time.time() - t < 0.1 or paused:
            vis.poll_events()
            vis.update_renderer()

        radar_frame_idx += 1

    break