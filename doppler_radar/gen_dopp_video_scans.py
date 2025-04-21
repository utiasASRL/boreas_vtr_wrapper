import os
import os.path as osp
import cv2
from utils.radar_utils import load_radar, radar_polar_to_cartesian, cfar_mask, extract_pc, point_to_cart_idx, mean_peaks_parallel_fast
from doppler_extractor import visualize_doppler, DopplerExtractor
import numpy as np
import matplotlib.pyplot as plt
import scipy
import time
from pyboreas import BoreasDataset
from sklearn import linear_model
from tqdm import tqdm
from matplotlib import rcParams
import matplotlib.transforms as transforms


rcParams['pdf.fonttype'] = 42
rcParams['ps.fonttype'] = 42

def main():
    # Load in all config parameters
    file_path = 'config/doppler_config.yaml'
    extractor = DopplerExtractor(file_path)

    az_start = 0
    az_end = 398
    # Tuning parameters for alignment
    min_range = 4   #m
    max_range = 200   #m    if 0 then use max range from radar data

    radar_res = 0.04381 # New radar data resolution
    #seq = ["boreas-2024-01-09-14-00"]
    seq = ["boreas-2024-11-02-14-14"]

    del_f = 893.0 * 10**6 # Hz
    df_dt = del_f * 1600 # Hz / s
    f_t = 76.04 * 10**9 # Hz
    beta_corr_fact = 0.944
    beta_up = beta_corr_fact*(f_t + del_f/2) / df_dt # Hz / s
    beta_down = -beta_up

    z_q = 2.5
    sigma_gauss = 15

    max_ransac_iter = 100
    ransac_threshold = 6.0

    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    seq_path = osp.join(dataset_dir, seq[0])
    rad_seq_path = osp.join(seq_path, 'radar')
    cam_seq_path = osp.join(seq_path, 'camera')
    img_list = sorted(os.listdir(rad_seq_path))
    if (osp.exists(cam_seq_path)): 
        cam_list = sorted(os.listdir(cam_seq_path))
        # Remove '.img' from all cam_list
        cam_times = [int(cam[:-4]) for cam in cam_list]
        cam_times = np.asarray(cam_times)

    bd = BoreasDataset(dataset_dir, split = None, verbose=False)
    bd_seq = bd.get_seq_from_ID(seq[0])
    vel_gt = np.asarray([frame.body_rate for frame in bd_seq.radar_frames]).squeeze(-1)
    time_gt = np.asarray([frame.timestamp_micro for frame in bd_seq.radar_frames])

    cart_dir = osp.join(rad_seq_path, 'cart')
    if not osp.exists(cart_dir): os.makedirs(cart_dir)
    cart_inv_dir = osp.join(rad_seq_path, 'cart_inv')
    if not osp.exists(cart_inv_dir): os.makedirs(cart_inv_dir)
    dopp_gt_dir = osp.join(rad_seq_path, 'dopp_gt')
    if not osp.exists(dopp_gt_dir): os.makedirs(dopp_gt_dir)
    dopp_est_dir = osp.join(rad_seq_path, 'dopp_est')
    if not osp.exists(dopp_est_dir): os.makedirs(dopp_est_dir)
    dopp_ransac_dir = osp.join(rad_seq_path, 'dopp_ransac')
    if not osp.exists(dopp_ransac_dir): os.makedirs(dopp_ransac_dir)
    full_img_dir = osp.join(rad_seq_path, 'full_img')
    if not osp.exists(full_img_dir): os.makedirs(full_img_dir)

    for img_name in img_list:
        if '.png' not in img_name:
            continue
        print(img_name)
        img = img_name[:-4]
        
        img_idx = img_list.index(img + '.png')
        img_path = osp.join(rad_seq_path, img_list[img_idx])

        gt_idx = np.where(time_gt == int(img))[0][0]

        if (gt_idx % 50 == 0):
            print(gt_idx)
        
        # Load in raw radar data
        radar_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        fft_data, azimuths, timestamps = load_radar(radar_img)
        fft_data[:, :42] = 0

        max_range_pix = int(max_range / radar_res)
        min_range_pix = int(min_range / radar_res)
        if max_range_pix == 0:
            max_range_pix = fft_data.shape[1]
            max_range = max_range_pix * radar_res

        # Get CFAR mask
        cfar_fft_up = cfar_mask(np.expand_dims(fft_data, 0), radar_res, a_thresh=0.7, b_thresh=0.23, maxr=max_range)
        cfar_fft_down = cfar_mask(np.expand_dims(fft_data, 0), radar_res, a_thresh=0.7, b_thresh=0.23, maxr=max_range)

        cfar_fft = np.zeros((1, fft_data.shape[0], fft_data.shape[1]))
        cfar_fft[0, ::2, :] = cfar_fft_up[0, ::2, :]
        cfar_fft[0, 1::2, :] = cfar_fft_down[0, 1::2, :]

        # Extract vehicle velocity estimates at each azimuth timestamp
        v_vehicle_gt = vel_gt[gt_idx, 0:3]

        # Compute ground truth velocity for each azimuth
        u_gt = v_vehicle_gt[0] * np.cos(azimuths) + v_vehicle_gt[1] * np.sin(azimuths)

        # Plot the FFT data
        fft_data_marked = fft_data.copy()
        fft_data_marked = np.expand_dims(fft_data_marked, axis=2).repeat(3, axis=2)
        fft_data_marked[az_start-1, :, :] = [1, 0, 0]
        fft_data_marked[az_end+1, :, :] = [1, 0, 0]

        u_scan = fft_data * np.nan
        u_az = azimuths * np.nan
        u_az_gt = azimuths * 0.0
        fft_data_use = fft_data.copy()

        for idx, ii in enumerate(range(az_start, az_end+1)):
            # Load in current azimuth data and process
            az_i = fft_data_use[ii, min_range_pix:max_range_pix]
            az_i = cen_filter(az_i, z_q, sigma_gauss, plot=False)

            # Load in next azimuth
            az_i_jj = fft_data_use[ii+1, min_range_pix:max_range_pix]
            az_i_jj = cen_filter(az_i_jj, z_q, sigma_gauss, plot=False)

            # Check if current azimuth has useable return
            # if np.all(az_i == 0) or np.all(az_i_jj == 0):
            #     u_i = np.nan
            #     del_r = np.nan
            #     del_r_pix = np.nan
            # else:
            corr_jj_i = scipy.signal.correlate(az_i, az_i_jj, mode='full', method='fft')
            del_r_jj = -(np.argmax(corr_jj_i) - (len(az_i) - 1))/2
            del_r = del_r_jj * radar_res

            if ii % 2 == 0:
                u_i = del_r / beta_up
            else:
                u_i = del_r / beta_down

            if np.abs(u_i) > 30:
                u_i = 30

            u_scan[ii, :] = u_i
            u_az[ii] = u_i
            u_az_gt[ii] = u_gt[ii]

        # Run RANSAC to find inlier u_az values
        az_use = azimuths[np.isnan(u_az) == False]
        u_az_use = u_az[np.isnan(u_az) == False]
        b = u_az_use.copy()
        A = np.cos(az_use)[:, np.newaxis]

        best_num_inlier = 0
        for iter in range(max_ransac_iter):
            # Randomly sample 2 points
            idx = np.random.choice(A.shape[0], 2, replace=False)
            A_sample = A[idx]
            b_sample = b[idx]
            # Fit line to sample
            model = linear_model.LinearRegression(fit_intercept=False)
            model.fit(A_sample, b_sample)

            if np.linalg.norm(model.coef_) - np.linalg.norm(v_vehicle_gt[0:2]) > 1.0:
                iter = iter - 1
                continue

            # Compute residuals
            residuals = A @ model.coef_ - b
            # Compute inliers
            num_inlier = np.sum(np.abs(residuals) < ransac_threshold)
            if num_inlier > best_num_inlier:
                best_num_inlier = num_inlier
                best_model = model
                best_residuals = residuals
                best_inlier_mask = np.abs(residuals) < ransac_threshold

        # print(best_num_inlier)
        inlier_mask = best_inlier_mask

        #inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        u_az_use[outlier_mask] = np.nan
        u_az_ransac = u_az.copy()
        u_az_ransac[np.isnan(u_az) == False] = u_az_use
        A = A[inlier_mask]
        b = b[inlier_mask]
        vel_est = scipy.optimize.least_squares(vel_est_err, v_vehicle_gt[0:1], args=(A, b), loss='cauchy', f_scale=0.6)

        if (np.linalg.norm(v_vehicle_gt) > 0.5 and np.linalg.norm(vel_est.x) > 0.5):
            if (np.abs(vel_est.x[0] - v_vehicle_gt[0]) > np.abs(vel_est.x[0] + v_vehicle_gt[0])):
                print("Flipping chirp type for sequence {} at frame {} with gt velocity {} and estimated velocity {}".format(seq[0], img, v_vehicle_gt[0], vel_est.x[0]))
                u_az_ransac = -u_az_ransac
                u_az = -u_az

        pc_RANSAC = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az_ransac, 0))
        pc_RANSAC = pc_RANSAC[0]

        # Find CFAR mask for fft data
        pc = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az, 0))
        pc = pc[0]
        pc_gt = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az_gt, 0))
        pc_gt = pc_gt[0]


        # Plot cartesian with overlaid points
        radar_cart_img = radar_polar_to_cartesian(fft_data, azimuths, radar_res,
                                                interpolate_crossover=True, fix_wobble=True)

        fig = plt.figure()
        plt.imshow(radar_cart_img, cmap='gray')
        plt.gca().set_axis_off()
        plt.savefig(osp.join(cart_dir, img + '.png'), bbox_inches='tight', pad_inches = 0)
        plt.close()

        fig = plt.figure()
        plt.gca().set_axis_off()
        visualize_doppler(radar_cart_img, pc_gt, start_fig=False, show_colourbar=False)
        plt.savefig(osp.join(dopp_gt_dir, img + '.png'), bbox_inches='tight', pad_inches = 0)
        plt.close()

        fig = plt.figure()
        plt.gca().set_axis_off()
        visualize_doppler(radar_cart_img, pc, start_fig=False, show_colourbar=False)
        plt.savefig(osp.join(dopp_est_dir, img + '.png'), bbox_inches='tight', pad_inches = 0)
        plt.close()

        fig = plt.figure()
        plt.gca().set_axis_off()
        visualize_doppler(radar_cart_img, pc_RANSAC, start_fig=False, show_colourbar=False)
        plt.savefig(osp.join(dopp_ransac_dir, img + '.png'), bbox_inches='tight', pad_inches = 0)
        plt.close()

        # Plot everything side-by-side
        if (osp.exists(cam_seq_path)):
            # Find nearest camera image
            cam_idx = np.argmin(np.abs(cam_times - int(img)))
            # Load in camera image
            camera_img = cv2.imread(osp.join(cam_seq_path, str(cam_list[cam_idx])))
            radar_height = radar_cart_img.shape[0]
            camera_img = cv2.resize(camera_img, (radar_height, radar_height))
            camera_img = np.flip(camera_img, axis=-1) 

        # Load in dopp_gt_dir image
        dopp_gt_img = cv2.imread(osp.join(dopp_gt_dir, img + '.png'))
        dopp_gt_img = np.flip(dopp_gt_img, axis=-1) 
        dopp_gt_img = cv2.rotate(dopp_gt_img, cv2.ROTATE_90_CLOCKWISE)
        # Load in dopp_est_dir image
        dopp_est_img = cv2.imread(osp.join(dopp_est_dir, img + '.png'))
        dopp_est_img = np.flip(dopp_est_img, axis=-1) 
        dopp_est_img = cv2.rotate(dopp_est_img, cv2.ROTATE_90_CLOCKWISE)
        # Load in dopp_ransac_dir image
        dopp_ransac_img = cv2.imread(osp.join(dopp_ransac_dir, img + '.png'))
        dopp_ransac_img = np.flip(dopp_ransac_img, axis=-1) 
        dopp_ransac_img = cv2.rotate(dopp_ransac_img, cv2.ROTATE_90_CLOCKWISE)

        figure = plt.figure()
        # Plot camera image
        sub1 = plt.subplot(1, 4, 1)
        plt.imshow(camera_img)
        sub1.axes.get_xaxis().set_ticks([])
        sub1.axes.get_yaxis().set_ticks([])

        # Plot gt doppler
        sub2 = plt.subplot(1, 4, 2)
        plt.imshow(dopp_gt_img)
        plt.title(r"Ground Truth", fontsize=10)
        sub2.axes.get_xaxis().set_ticks([])
        sub2.axes.get_yaxis().set_ticks([])

        # Plot estimated doppler
        sub3 = plt.subplot(1, 4, 3)
        plt.imshow(dopp_est_img)
        plt.title(r"Raw Estimates", fontsize=10)
        sub3.axes.get_xaxis().set_ticks([])
        sub3.axes.get_yaxis().set_ticks([])

        # Plot RANSAC doppler
        sub4 = plt.subplot(1, 4, 4)
        plt.imshow(dopp_ransac_img)
        plt.title(r"RANSAC Estimates", fontsize=10)
        sub4.axes.get_xaxis().set_ticks([])
        sub4.axes.get_yaxis().set_ticks([])

        plt.subplots_adjust(left=0.0,
                            bottom=0.0, 
                            right=1.0, 
                            top=1.0, 
                            wspace=0.0, 
                            hspace=0.0)
        plt.savefig(osp.join(full_img_dir, img + '.png'), bbox_inches='tight', pad_inches = 0)
        plt.close()

    # Generate videos
    print("Generating videos")
    generate_video(cart_dir, rad_seq_path, 'radar', False)
    generate_video(cart_inv_dir, rad_seq_path, 'radar_inv', False)
    generate_video(full_img_dir, rad_seq_path, 'full_img', rotate=False, H=420, W=1600)
    generate_video(dopp_gt_dir, rad_seq_path, 'dopp_gt', True)
    generate_video(dopp_est_dir, rad_seq_path, 'dopp_est', True)
    generate_video(dopp_ransac_dir, rad_seq_path, 'dopp_ransac', True)


def generate_video(root_dir, output_dir, video_name, rotate=False, H=400, W=400):
    files = os.listdir(root_dir, )
    img_files = [f for f in files if 'png' in f]
    img_files.sort()

    upper_crop = 0
    h_before_resize = 1377
    frame_rate = 30

    out = cv2.VideoWriter(output_dir + '/' + video_name + '.mp4', cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), frame_rate, (W, H))

    for i in tqdm(range(len(img_files)), desc="Extracting"):
        frame = cv2.imread(root_dir + '/' + img_files[i])
        frame = frame[upper_crop:upper_crop+h_before_resize]
        frame = cv2.resize(frame, (W, H))
        if rotate:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        out.write(frame)

    out.release()
    cv2.destroyAllWindows()

def vel_est_err(x, A, b):
    return b - A @ x

def cen_filter(s, z_q, sigma_gauss, plot=False, time=False):
    if time: tic = time.time()
    #q = s - scipy.ndimage.median_filter(s, size=w_median)
    q = s - np.mean(s)
    if time:
        toc = time.time()
        print("Median filter time: {}".format(toc-tic))
    
    """
    # Thank you stackoverflow: https://stackoverflow.com/questions/56246970/how-to-apply-a-binomial-low-pass-filter-to-data-in-a-numpy-array
    if time:tic = time.time()
    p = np.zeros_like(q)
    p = scipy.signal.convolve(q, b, mode='same')
    if time:
        toc = time.time()
        print("Binomial filter time: {}".format(toc-tic))
    """

    # Create 1D Gaussian Filter
    p = scipy.ndimage.gaussian_filter(q, sigma_gauss, truncate=3.0)

    # Assume values of q below 0 are Gaussian noise with mean 0 and std sigma_q
    if time: tic = time.time()
    sigma_q = np.std(q[q < 0])
    q_dist = scipy.stats.norm(0, sigma_q)
    if time:
        toc = time.time()
        print("Gaussian noise time: {}".format(toc-tic))

    # Scale p by q_dist
    if time: tic = time.time()
    y_low_f = p * (1 - q_dist.pdf(p)/ q_dist.pdf(0))
    y_low_f[y_low_f < 0] = 0
    
    y_high_f = y_low_f + (q - p) * (1 - q_dist.pdf(q - p)/ q_dist.pdf(0))
    y_high_f[y_high_f < 0] = 0
    
    y_thres = y_high_f.copy()
    y_thres[y_high_f < z_q*sigma_q] = 0

    if time:
        toc = time.time()
        print("Scaling time: {}".format(toc-tic))
    if plot:
        fig = plt.figure()
        axs = fig.subplots(6, 1)
        axs[0].plot(s, label='Original')
        axs[0].set_title("Original")
        axs[1].plot(q, label='Median filtered')
        axs[1].set_title("Median filtered")
        axs[2].plot(p, label='Binomial filtered')
        axs[2].set_title("Binomial filtered")
        axs[3].plot(y_low_f, label='Scaled Low-F')
        axs[3].set_title("Scaled Low-F")
        axs[4].plot(y_high_f, label='Scaled High-F')
        axs[4].set_title("Scaled High-F")
        axs[5].plot(y_thres, label='Thresholded')
        axs[5].set_title("Thresholded")

    return y_thres


if __name__ == "__main__":
    main()