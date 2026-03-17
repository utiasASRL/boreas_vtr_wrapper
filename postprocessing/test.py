import numpy as np
from pyboreas import BoreasDataset
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
import cv2

def cen_filter_1d(signal, sigma_gauss=15.0, z_q=2.5):
    # 1. Subtract mean
    mean_val = np.mean(signal)
    q = signal - mean_val
    
    # 2. Apply Gaussian filter 
    p = gaussian_filter1d(q, sigma=sigma_gauss, mode='reflect')
    
    # 3. Estimate TRUE noise sigma 
    neg_mask = q < 0
    count = np.sum(neg_mask)
    
    if count > 0:
        # BUG FIX: Removed the 0.1 multiplier!
        # The 2.0 correctly accounts for the fact that we are only looking at the negative half of the noise.
        sigma_q = 0.5 * np.sqrt(np.sum(2.0 * (q[neg_mask] ** 2)) / count)
    else:
        sigma_q = 0.034
        
    threshold = z_q * sigma_q
    
    # 4. Compute adaptive weighting
    eps = 1e-8 # Safety against division by zero
    pow_p = (p / (sigma_q + eps)) ** 2
    pow_qp = ((q - p) / (sigma_q + eps)) ** 2
    
    npp = np.exp(-0.5 * pow_p)
    nqp = np.exp(-0.5 * pow_qp)
    
    # Re-weight the signal
    y = q * (1.0 - nqp) + p * (nqp - npp)
    
    # 5. Zero out signal values less than threshold
    y[y <= threshold] = 0.0
    
    return y

def cen_filter_2d(polar_image, sigma_gauss=15.0, z_q=2.5, noise_scale=1.0):
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


# Grab the user's home directory
boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")

boreas_data = '/boreas_data' # TODO change to use environment variable
bd = BoreasDataset(boreas_data)

# Loop through each frame in order (odometry)
for seq in bd.sequences:
    print(f"SequenceID: {seq.ID}")
    
    for i in range(len(seq.radar_frames)):
        if i != 100:
            continue
        radar_frame = seq.get_radar(i)
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

            # if idx % 25 == 0:
            #     filtered_waveform = filtered_polar[idx]
            #     shifted_waveform = shifted_polar[idx]
            #     # filtered_waveform_shifted = shift(filtered_waveform)

            #     # 1. Initialize a figure (Width, Height in inches)
            #     plt.figure(figsize=(12, 5)) # Made it slightly wider for better 1D viewing

            #     # 2. Plot BOTH waveforms
            #     # Plot the raw data first. alpha=0.4 makes it semi-transparent
            #     plt.plot(shifted_waveform, color='blue', linewidth=1.0, alpha=0.4, label="Shifted Signal")
                
            #     # Plot the filtered data on top in a bold color
            #     plt.plot(filtered_waveform, color='red', linewidth=1.5, label="Cen Filtered")

            #     # 3. Format the graph 
            #     plt.title(f"Radar Beam Waveform - Azimuth {idx}")
            #     plt.xlabel("Range Bin (Array Index)")
            #     plt.ylabel("Reflectivity / Power")
            #     plt.grid(True, linestyle='--', alpha=0.6)
                
            #     # Show the legend so we know which line is which
            #     plt.legend(loc="upper right")

            #     # 4. Save the figure to your disk
            #     plt.savefig(f"waveforms_filtered/{idx}.png", dpi=300, bbox_inches='tight')

            #     # 5. Clear the memory! 
            #     plt.close()
        
        
        # print(radar_frame.chirp_type.shape)
        # print(radar_frame.body_rate)
        # print(seq.calib.radar_doppler_beta)
        # print(seq.calib.radar_offset)
        # print(radar_frame.resolution)
        # print(radar_frame.polar.shape)

        # cart_dir = Path(radar_frame.sensor_root) / "cart_filtered"
        # cart_dir.mkdir(parents=True, exist_ok=True) 
        # cart_path = cart_dir / f"{radar_frame.frame}.png"
        # cart_resolution = 0.2384
        # cart_pixel_width = 640
        
        filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)
        radar_frame.polar = filtered_polar
        print(radar_frame.frame)
        radar_frame.visualize()
        # cart = radar_frame.polar_to_cart(cart_resolution=cart_resolution, cart_pixel_width=cart_pixel_width)

        # cartesian_8bit = cv2.normalize(
        #     cart, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        # )

        # success = cv2.imwrite(str(cart_path), cartesian_8bit)
        # if success:
        #     print(f"Saved to {cart_path}")
        # else:
        #     print(f"ERROR: OpenCV failed to write to {cart_path}")

        radar_frame.unload_data()
        break
    break



