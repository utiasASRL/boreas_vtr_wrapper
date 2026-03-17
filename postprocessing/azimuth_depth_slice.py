import numpy as np
from pyboreas import BoreasDataset
import os
import time
import cv2
from pathlib import Path

from sensor_msgs_py.point_cloud2 import read_points
import open3d as o3d
from pylgmath import Transformation
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex, extract_points_from_vertex
import argparse


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



class DepthImage:
    def __init__(
        self, 
        elevation_res_deg=0.1, 
        azimuth_res_deg=0.1, 
        elevation_lower_deg=-15.0, 
        elevation_upper_deg=15.0, 
        azimuth_lower_deg=-180.0, 
        azimuth_upper_deg=180.0, 
        hfov_deg=60.0,
        vfov_deg=30.0,
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

# --- How to use it on your actual depth image ---
H = 300
W = 3600

file_path = "/boreas_data/boreas-2024-12-03-12-54/depth_arrays/1733248462079298.npy"

# 2. Load the array into memory
depth_array = np.load(file_path)
print(np.sum(depth_array))
depth_image = DepthImage()
depth_image.image = depth_array

target_azimuths = np.array([0.0, 30.0, 60.0])
patches = extract_patches_for_azimuths(depth_image, target_azimuths)
print(f"Extracted shape: {patches.shape} -> (N_patches, Height, Width)")

# --- Visualization of the First Patch ---
sample_patch = patches[0].copy()

min_range = 1.0
max_range = 200.0

# Save RGB colour depth map
vis_img_8bit = np.zeros_like(sample_patch, dtype=np.uint8)
valid_pixels = sample_patch > 0
linear_pct = 2 * (sample_patch[valid_pixels] - min_range) / (max_range - min_range)
scaled_depth = 255.0 * (1.0 - linear_pct)
vis_img_8bit[valid_pixels] = np.clip(scaled_depth, 0, 255).astype(np.uint8)

# Apply the Turbo colormap (Creates the RGB image)
color_depth = cv2.applyColorMap(vis_img_8bit, cv2.COLORMAP_TURBO)

# Set the empty background pixels back to pure black
color_depth[vis_img_8bit == 0] = [0, 0, 0]

cv2.imwrite("1733248462079298.png", color_depth)

# Since radar is upside down, also save flipped image
color_depth_flipped = cv2.flip(color_depth, 0)
cv2.imwrite("1733248462079298_flipped.png", color_depth_flipped)


# make it so it can take an array of azimuths (make sure they are all the same size)