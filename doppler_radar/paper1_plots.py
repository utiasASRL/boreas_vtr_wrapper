import os
import os.path as osp
import cv2
from utils.radar_utils import load_radar, radar_polar_to_cartesian, cfar_mask, extract_pc
from doppler_extractor import visualize_doppler, DopplerExtractor
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import scipy
import time
from pyboreas import BoreasDataset
from sklearn import linear_model
import matplotlib.gridspec as gridspec
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib import transforms
import yaml

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
matplotlib.rcParams["text.usetex"] = True


def load_yaml_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def main():
    # Seq for paper 1 plot
    seq = ["boreas-2024-01-09-14-00"]
    # old dynamic 1704826821486644
    img_list = ['1704827957816179' , '1704826819489279']    # Static, dynamic in static env

    # Load in all config parameters
    file_path = 'config/doppler_config.yaml'
    extractor = DopplerExtractor(file_path)
    config = load_yaml_config(file_path)

    # Still need to load in parameters since we're using more than just extractor for visualizations
    az_start = 0
    az_end = 399
    min_range = config['extraction']['signal']['min_range']   #m
    max_range = config['extraction']['signal']['max_range']   #m    if 0 then use max range from radar data
    radar_res = config['radar']['radar_res'] # New radar data resolution

    # Load in Boreas data and other data paths
    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    seq_path = osp.join(dataset_dir, seq[0])
    rad_seq_path = osp.join(seq_path, 'radar')

    bd = BoreasDataset(dataset_dir, split = None, verbose=False)
    bd_seq = bd.get_seq_from_ID(seq[0])
    vel_gt = np.asarray([frame.body_rate for frame in bd_seq.radar_frames]).squeeze(-1)
    time_gt = np.asarray([frame.timestamp_micro for frame in bd_seq.radar_frames])

    all_plot_data = []
    for img in img_list:
        img_path = osp.join(rad_seq_path, img + '.png')

        gt_idx = np.where(time_gt == int(img))[0][0]
        
        # Load in raw radar data
        radar_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        fft_data, azimuths, timestamps = load_radar(radar_img)
        fft_data[:, :42] = 0    # Zero out first ~2m to get rid of reflections from our car

        max_range_pix = int(max_range / radar_res)
        min_range_pix = int(min_range / radar_res)
        if max_range_pix == 0:
            max_range_pix = fft_data.shape[1]
            max_range = max_range_pix * radar_res

        # Extract vehicle velocity gt
        v_vehicle_gt = vel_gt[gt_idx, 0:3]

        u_az = azimuths * np.nan
        u_az_gt = azimuths * 0.0
        az_val = azimuths * 0.0
        for idx, ii in enumerate(range(az_start, az_end)):
            # Load in current azimuth data and process
            az_i_og = fft_data[ii, min_range_pix:max_range_pix].copy()

            # Load in next azimuth
            az_i_jj_og = fft_data[ii+1, min_range_pix:max_range_pix].copy()

            # Extract relative velocity between azimuths
            # Need minus because these scans are flipped in up/down chirps
            u_i, az_i_cen, az_i_jj_cen, del_r_jj = extractor.extract_single_doppler(fft_data[ii], fft_data[ii+1], up_chirp=(ii % 2 == 0), copy=True, return_everything=True)
            u_i = -u_i
            # Cap velocity estimates for visualization
            if np.abs(u_i) > 15:
                u_i = 15

            # Store results
            az_ii_iip1 = (azimuths[ii] + azimuths[ii+1])/2
            
            u_az[ii] = u_i
            u_az_gt[ii] = v_vehicle_gt[0] * np.cos(az_ii_iip1) + v_vehicle_gt[1] * np.sin(az_ii_iip1)            
            az_val[ii] = az_ii_iip1
            time_i = (timestamps[ii] + timestamps[ii+1])/2 - timestamps[0]

            if ii==155 and seq == ["boreas-2024-01-09-14-00"] and img == '1704826819489279':
                fig = plt.figure(figsize=(6, 2))
                plt.plot(az_i_og[:1510], label=r'$\mathrm{z}_i$', rasterized=True)
                plt.plot(az_i_jj_og[:1510], label=r'$\mathrm{z}_{i+1}$', rasterized=True)
                plt.xlabel(r'Range bin', size=18)
                plt.ylabel(r'Amp.', size=18)
                plt.legend(fontsize=16, loc='upper right')
                plt.tight_layout()
                plt.savefig(r"./results/raw_signal.pdf")

                fig = plt.figure(figsize=(6,2))
                plt.plot(az_i_cen[:1510], label=r'$\mathrm{z}_i$', rasterized=True)
                plt.plot(az_i_jj_cen[:1510], label=r'$\mathrm{z}_{i+1}$', rasterized=True)
                plt.xlabel(r'Range bin', size=18)
                plt.ylabel(r'Amp.', size=18)
                plt.legend(fontsize=16, loc='upper right')
                plt.tight_layout()
                plt.savefig(r"./results/cen_signal.pdf")
                az_i_jj_roll_est = np.roll(az_i_jj_cen, -int(del_r_jj))

                fig = plt.figure(figsize=(6, 2))
                plt.plot(az_i_cen[:1510], label=r'$\mathrm{z}_i$', rasterized=True)
                plt.plot(az_i_jj_roll_est[:1510], label=r'$\mathrm{z}_{i+1}$', rasterized=True)
                plt.xlabel(r'Range bin', size=18)
                plt.ylabel(r'Amp.', size=18)
                plt.legend(fontsize=16, loc='upper right')
                plt.tight_layout()
                plt.title(ii)
                plt.savefig(r"./results/cc_signal.pdf")

        percent_0 = np.sum(np.abs(u_az[~np.isnan(u_az)]) < 0.5) / len(u_az[~np.isnan(u_az)])
        print("Percent of 0 values: ", percent_0)

        # Run RANSAC to isolate inliers
        u_ransac, az_ransac = extractor.ransac(u_az, az_val)

        # Estimate velocity
        vel_est = extractor.estimate_vel_from_radial(u_ransac, az_ransac, v_init=vel_gt[gt_idx, 0:2])
    
        # Compute azimuth-wise RMSE
        az_rmse = extractor.compute_RMSE(u_ransac, az_ransac, v_vehicle_gt)

        print("azimuth RMSE: ", az_rmse)
        print("Gt vel: ", vel_gt[gt_idx, 0:3])
        print("Est fwd: ", vel_est)
        print("Err fwd: ", vel_est.squeeze() - vel_gt[gt_idx, 0:2].squeeze())

        # Get CFAR data to use for pointcloud overlay
        cfar_fft_up = cfar_mask(np.expand_dims(fft_data, 0), radar_res, a_thresh=0.8, b_thresh=0.23, maxr=max_range)
        cfar_fft_down = cfar_mask(np.expand_dims(fft_data, 0), radar_res, a_thresh=0.8, b_thresh=0.23, maxr=max_range)
        cfar_fft = np.zeros((1, fft_data.shape[0], fft_data.shape[1]))
        cfar_fft[0, ::2, :] = cfar_fft_up[0, ::2, :]
        cfar_fft[0, 1::2, :] = cfar_fft_down[0, 1::2, :]

        # Extract groundtruth overlayed pointcloud
        pc_gt = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az_gt, 0))
        pc_gt = pc_gt[0]
        # Extract raw extracted velocity pointcloud
        pc = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az, 0))
        pc = pc[0]
        # Extract RANSAC pointcloud
        pc_RANSAC = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_ransac, 0))
        pc_RANSAC = pc_RANSAC[0]

        # Plot RANSAC results
        b = u_az[~np.isnan(u_az)]
        A = np.hstack((np.cos(az_val[~np.isnan(u_az)])[:, np.newaxis], np.sin(az_val[~np.isnan(u_az)])[:, np.newaxis]))
        b_ransac = u_ransac[~np.isnan(u_ransac)]
        A_ransac = np.hstack((np.cos(az_ransac[~np.isnan(u_ransac)])[:, np.newaxis], np.sin(az_ransac[~np.isnan(u_ransac)])[:, np.newaxis]))

        fig = plt.figure(figsize=(6,4))
        plt.plot(A[:,0], b, 'ro', label=r'$\mathbf{u}$', rasterized=True)
        plt.plot(A_ransac[:, 0], b_ransac, 'o', c='C0', label=r'$\mathbf{u}_R$', rasterized=True)
        plt.legend(fontsize=18)
        plt.xlabel(r'$\cos(\phi)$', size=18)
        plt.ylabel(r'$u$ [m/s]', size=18)
        plt.tight_layout()
        plt.savefig(r"./results/y_vs_cos.pdf")

        # Save cartesian with overlaid points
        radar_cart_img = radar_polar_to_cartesian(fft_data, azimuths, radar_res,
                                                interpolate_crossover=True, fix_wobble=True)
        
        plt_data = {'timestamp': img, 'radar_cart_img': radar_cart_img, 'pc_gt': pc_gt, 'pc': pc, 'pc_RANSAC': pc_RANSAC}
        all_plot_data.append(plt_data)

    plot_vel_overlay(all_plot_data)

def plot_vel_overlay(all_plot_data):

    matplotlib.rcParams['axes.linewidth'] = 5.0
    matplotlib.rcParams['axes.edgecolor'] = 'black'

    # Plot velocity overlays
    gridspec_width = {'width_ratios': [5, 5]}
    num_data_points = len(all_plot_data)
    num_row = 3
    fig, axs = plt.subplots(num_row, num_data_points, figsize=(num_data_points*5.0, 15), gridspec_kw=gridspec_width)

    for idx, data in enumerate(all_plot_data):
        timestamp = data['timestamp']
        radar_cart_img = data['radar_cart_img']
        pc_gt = data['pc_gt']
        pc = data['pc']
        pc_RANSAC = data['pc_RANSAC']

        # Rotate everything to point upwards
        radar_cart_img = np.rot90(radar_cart_img, 3)
        for pc_mod in [pc_gt, pc, pc_RANSAC]:
            pc_x = pc_mod[:, 0].copy()
            pc_y = pc_mod[:, 1].copy()
            pc_mod[:, 0] = -pc_y
            pc_mod[:, 1] = pc_x

        motion_type = 'dynamic'  # 'static' or 'dynamic'
        if timestamp == '1704827957816179':
            motion_type = 'static'
        elif timestamp == '1704826819489279':
            motion_type = 'dynamic'  # 'static' or 'dynamic'

        sub1 = plt.subplot(num_row, num_data_points, idx + 1)
        visualize_doppler(radar_cart_img, pc_gt, start_fig=False, show_colourbar=False)
        plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2-7.5, 0, 15, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)

        if motion_type == 'static': plt.ylabel(r"Groundtruth", size=18, fontdict={'family': 'serif'})
        sub2 = plt.subplot(num_row, num_data_points, idx + num_data_points + 1)
        visualize_doppler(radar_cart_img, pc, start_fig=False, show_colourbar=False)
        plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2-7.5, 0, 15, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)
        sub2.spines[['top']].set_visible(False)

        if motion_type == 'static':  plt.ylabel(r"Raw Estimates", size=18, fontdict={'family': 'serif'})
        sub3 = plt.subplot(num_row, num_data_points, idx + 2*num_data_points + 1)
        img = visualize_doppler(radar_cart_img, pc_RANSAC, start_fig=False, show_colourbar=False)
        plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2-7.5, 0, 15, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)
        sub3.spines[['top']].set_visible(False)

        if motion_type == 'static': plt.ylabel(r"RANSAC Estimates", size=18, fontdict={'family': 'serif'})
        if motion_type == 'static': axs[0, idx].set_title(r'Static', size=18, fontdict={'family': 'serif'}, pad=10)
        elif motion_type == 'dynamic': axs[0, idx].set_title(r'Dynamic', size=18, fontdict={'family': 'serif'}, pad=10)

    plt.subplots_adjust(left=0.0,
                        bottom=0.0, 
                        right=1.0, 
                        top=1.0, 
                        wspace=0.0, 
                        hspace=0.0)
    plt.savefig(r"./results/velocity_overlay.pdf", bbox_inches='tight', transparent=True)

    # Now save colorbar separately
    fig = plt.figure()
    fig = plt.figure(figsize=(0.2, 10))
    ax = fig.add_axes([0.05, 1.0, 1.0, 1.0])

    cb = matplotlib.colorbar.ColorbarBase(ax, orientation='vertical', 
                                cmap='bwr_r', norm=matplotlib.colors.Normalize(vmin=-10, vmax=10))
    cb.set_label(r'Velocity [m/s]', size=16, fontdict={'family': 'serif'})
    cb.ax.tick_params(labelsize=16)
    plt.savefig(r"./results/colorbar.pdf", bbox_inches='tight', transparent=True)


if __name__ == "__main__":
    main()