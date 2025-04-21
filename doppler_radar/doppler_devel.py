import os
import os.path as osp
import cv2
from utils.radar_utils import load_radar, radar_polar_to_cartesian, cfar_mask, extract_pc, point_to_cart_idx, mean_peaks_parallel_fast
from doppler_radar.doppler_extractor import cen_filter, visualize_doppler, estimate_doppler, cross_correlate
import numpy as np
import matplotlib.pyplot as plt
import scipy
import time
from pyboreas import BoreasDataset
from sklearn import linear_model

def main():
    az_start = 1
    az_end = 398
    # Tuning parameters for alignment
    min_range = 3   #m
    max_range = 100   #m    if 0 then use max range from radar data
    max_del_r_diff = 100 # pixels (max difference between del_r_m1 and del_r_p1)

    plot = False

    radar_res = 0.04381 # New radar data resolution
    #seq = ["boreas-2023-10-12-11-41"]
    #seq = ["boreas-2023-12-15-10-27"]
    #seq = ["boreas-2023-12-15-11-30"]
    #seq = ["boreas-2023-12-15-13-37"]
    seq = ["boreas-2024-01-09-14-00"]
    #seq = ["boreas-2024-01-23-11-45"]
    #seq = ['boreas-2024-01-23-12-15']
    #seq = ["boreas-2024-01-23-12-32"]
    #seq = ['boreas-2024-02-13-16-13']
    #seq = ["boreas-2024-02-29-12-31"]
    #seq = ["boreas-2024-01-23-12-15"]
    # seq = ["boreas-2024-01-25-11-44"]
    # seq = ["boreas-2024-02-13-15-50"]

    # Imgs for boreas-2024-01-09-14-00
    img = '1704826804513623'    # Dynamic in static env
    #img = '1704826804763289'     # Static
    #img = '1704826998456130'
    #img = '1704827625137523'
    #img = '1704826824981643'
    #img = '1704826819739042'
    #img = '1704827017923487'    # Static
    #img = '1704826810003820'    # Low speed
    #img = '1704826828226180'
    #img = '1704826946541877' # Static
    #img = '1704826858427188'
    #img = '1704826947540370'

    # Imgs for boreas-2024-01-23-11-45
    #img = '1706028336198174' # Static
    #img = '1706028500898879'

    # Imgs for boreas-2024-01-23-12-15
    #img = '1706030307179634'
    #img = '1706030756480605'

    # Imgs for boreas-2023-12-15-11-30
    #img = '1702658678982396' # Static
    #img = '1702658641977570' # Dynamic
    #img = '1702657896412285' # Dynamic with objects
    #img = '1702657890663543'  # Challenging
    #img = '1702657893912262'
    #img = '1702657911662543'

    # Imgs for boreas-2024-02-13-16-13
    #img = '1707858840775564' # Static
    #img = '1707859013313117' # Hwy timestamp

    # Imgs for boreas-2023-12-15-10-27
    #img = '1702654028515878'    # Static
    #img = '1702654043003195'    # Moving in static env

    # Imgs for boreas-2023-10-12-11-41
    #img = '1697125305701698'    # Static 200-201
    #img = '1697125321174634'    # Moving in static env 180-185
    #img = '1697125932173377'   # Dynamic object
    #img = '1697126391508675'
    #img = '1697125496911436'
    #img = '1697126217990594'
    #img = '1697125562826942'

    # Imgs for boreas-2024-02-29-12-31
    #img = '1709228090724164'

    # Imgs for boreas-2024-01-23-12-15 
    #img = '1706030614211080'    # used for extractor diagram

    img = '1706201704318657'
    #img = '1706201704568361'
    img = '1706201704818447'
    img = '1707857846014902'
    img = '1707857845016317'
    img = '1707857842269449'

    del_f = 893.0 * 10**6 # Hz
    df_dt = del_f * 1600 # Hz / s
    f_t = 76.04 * 10**9 # Hz
    beta_corr_fact = 0.95
    beta_up = -beta_corr_fact*(f_t + del_f/2) / df_dt # Hz / s
    #beta_down = -beta_corr_fact*(f_t - del_f/2) / df_dt # Hz / s
    beta_down = -beta_up

    entire_scan_simultaneous = False
    use_gt_vel_shift = False
    join_corr = False
    subtract_mean = False
    normalize = False
    square_signal = False
    exp_signal = False
    remove_mean = False
    upsample = False
    cfar_enabled = False
    moving_average = False
    avg_with_neighbours = False
    filter_with_neighbours = False
    cen_filtering = True
    db_scale = False
    lowpass = False

    w_median = 100
    w_binom = 20
    z_q = 2.5
    sigma_gauss = 15
    num_neighbours = 1
    upsample_factor = 100
    ma_n = 40
    b = (np.poly1d([0.5, 0.5])**w_binom).coeffs


    b, a = scipy.signal.butter(12, 0.250)

    if not upsample: upsample_factor = 1

    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    seq_path = osp.join(dataset_dir, seq[0])
    rad_seq_path = osp.join(seq_path, 'radar')
    img_list = sorted(os.listdir(rad_seq_path))
    img_idx = img_list.index(img + '.png')
    img_path = osp.join(rad_seq_path, img_list[img_idx])

    bd = BoreasDataset(dataset_dir, split = None, verbose=False)
    bd_seq = bd.get_seq_from_ID(seq[0])
    vel_gt = np.asarray([frame.body_rate for frame in bd_seq.radar_frames])
    print(vel_gt.shape)
    fdsa
    time_gt = np.asarray([frame.timestamp_micro for frame in bd_seq.radar_frames])
    gt_idx = np.where(time_gt == int(img))[0][0]
    poses = np.asarray([frame.pose for frame in bd_seq.radar_frames])

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
    cfar_fft_up = cfar_mask(np.expand_dims(fft_data, 0), radar_res, a_thresh=0.5, b_thresh=0.27, maxr=max_range)
    cfar_fft_down = cfar_mask(np.expand_dims(fft_data, 0), radar_res, a_thresh=0.5, b_thresh=0.19, maxr=max_range)

    cfar_fft = np.zeros((1, fft_data.shape[0], fft_data.shape[1]))
    cfar_fft[0, ::2, :] = cfar_fft_up[0, ::2, :]
    cfar_fft[0, 1::2, :] = cfar_fft_down[0, 1::2, :]

    # Gt vel stuff
    # Compute ground truth
    # Extract velocity and time from +/- 1 frame
    t_span = time_gt[gt_idx-1:gt_idx+2]
    v_span = vel_gt[gt_idx-1:gt_idx+2]

    # Fit spline to data
    spline = scipy.interpolate.make_interp_spline(t_span, v_span, k=2)
    # Extract vehicle velocity estimates at each azimuth timestamp
    v_vehicle_gt = spline(timestamps)

    """
    fig = plt.figure()
    plt.plot(time_gt/1000, vel_gt[:, 0])
    plt.plot(time_gt/1000, vel_gt[:, 1])
    plt.plot(time_gt/1000, vel_gt[:, 2])
    plt.legend(['x', 'y', 'z'])
    plt.show()
    print(v_vehicle_gt.shape)
    fads
    """
    # Compute ground truth velocity for each azimuth
    u_gt = v_vehicle_gt[:, 0] * np.cos(azimuths) + v_vehicle_gt[:, 1] * np.sin(azimuths)

    # Plot the FFT data
    num_az = az_end - az_start + 1

    fft_data_marked = fft_data.copy()
    fft_data_marked = np.expand_dims(fft_data_marked, axis=2).repeat(3, axis=2)
    fft_data_marked[az_start-1, :, :] = [1, 0, 0]
    fft_data_marked[az_end+1, :, :] = [1, 0, 0]

    #fig = plt.figure()
    #plt.imshow(fft_data_marked)

    radar_cart_img_marked = radar_polar_to_cartesian(fft_data_marked, azimuths, radar_res, interpolate_crossover=True, fix_wobble=True)

    if plot: fig, axs = plt.subplots(num_az, 1)

    u_scan = fft_data * np.nan
    u_az = azimuths * np.nan
    u_az_gt = azimuths * 0.0
    fft_range_corr = fft_data.copy()
    fft_data_use = fft_data.copy()
    #if cen_filtering:
    #    b = np.expand_dims(b, 0).repeat(fft_data_use.shape[0], axis=0)
    #    #fft_data_use = cen_filter_mult(fft_data_use, w_median, b, z_q, plot=False)
    #    q = fft_data_use[:, min_range_pix:max_range_pix] - scipy.ndimage.median_filter(fft_data_use[:, min_range_pix:max_range_pix], size=w_median)
    #    # Thank you stackoverflow: https://stackoverflow.com/questions/56246970/how-to-apply-a-binomial-low-pass-filter-to-data-in-a-numpy-array
    #   p = np.zeros_like(q)
    #   p = scipy.signal.convolve(q, b, mode='same')
    

    var_u = []
    err_u = []
    del_r_est_list = []
    del_r_gt_list = []
    del_r_az_list = []
    var_window = 2
    NEES_list = []

    if entire_scan_simultaneous:
        fft_up = fft_data[::2, min_range_pix:max_range_pix]
        fft_down = fft_data[1::2, min_range_pix:max_range_pix]

        # Compute correlation between the two
        corr_up_down = scipy.signal.correlate(fft_up, fft_down, mode='full', method='fft')

        print(corr_up_down.shape)

        # Compute del_r from correlation
        del_r = (np.argmax(corr_up_down) - (fft_up.shape[1] - 1))/2

        # Compute velocity from del_r
        u = del_r * radar_res / beta_up
        u_gt = vel_gt[gt_idx, 0]
        print("u: ", u)
        print("u_gt: ", u_gt)

        del_r_gt = (u_gt * beta_up) / radar_res


        fig = plt.figure()
        plt.plot(corr_up_down)
        # Plot red line at del_r_gt and blue line at jj_shift
        #plt.axvline(x=np.argmax(corr_up_down), color='b')
        #plt.axvline(x=del_r_gt + (fft_up.shape[1] - 1), color='r')

        plt.show()

        afds



    else:
        for idx, ii in enumerate(range(az_start, az_end+1)):
            #angle = np.pi/2.5
            #if azimuths[ii] > angle and azimuths[ii] < np.pi - angle:
            #    continue
            #if azimuths[ii] < 2*np.pi - angle and azimuths[ii] > np.pi + angle:
            #    continue

            #if np.cos(azimuths[ii]) > 0.0:
            #    continue

            #if azimuths[ii] > np.pi + np.pi/3:
            #    continue
            #if azimuths[ii] < np.pi - np.pi/3:
            #    continue

            # if ii % 2 == 0: continue

            # Load in current azimuth data and process
            az_i = fft_data_use[ii, min_range_pix:max_range_pix]
            az_i_og = az_i.copy()

            if avg_with_neighbours or filter_with_neighbours:
                az_i_m2 = fft_data_use[ii-2, min_range_pix:max_range_pix]
                az_i_p2 = fft_data_use[ii+2, min_range_pix:max_range_pix]

                if filter_with_neighbours:
                    corr_i_m2 = scipy.signal.correlate(az_i, az_i_m2, mode='full', method='fft')
                    corr_i_p2 = scipy.signal.correlate(az_i, az_i_p2, mode='full', method='fft')
                    # Divide by 2 to get del_r to true range
                    del_r_m2 = (np.argmax(corr_i_m2) - (len(az_i) - 1))/2
                    del_r_p2 = (np.argmax(corr_i_p2) - (len(az_i) - 1))/2

                    if abs(del_r_m2) > 1 or abs(del_r_p2) > 1:
                        continue

                if avg_with_neighbours:
                    az_i = (az_i + az_i_m2 + az_i_p2) / 3

            if subtract_mean: az_i = az_i - np.mean(az_i)
            if normalize: az_i = az_i / np.max(az_i)
            if remove_mean: az_i[az_i < 2*np.mean(fft_data_use[ii])] = 0.0
            if square_signal: az_i = az_i**2
            if exp_signal: az_i = np.exp(az_i)
            if upsample: az_i = scipy.signal.resample(az_i, upsample_factor*len(az_i))

            if moving_average:
                ret = np.cumsum(az_i)
                ret[ma_n:] = ret[ma_n:] - ret[:-ma_n]
                az_i = ret[ma_n - 1:] / ma_n

            if cfar_enabled:
                cfar_i = cfar_fft[0, ii, min_range_pix:max_range_pix]
                if upsample:
                    cfar_i = scipy.signal.resample(cfar_i, upsample_factor*len(cfar_i))
                az_i = az_i * cfar_i

            if cen_filtering:
                az_i = cen_filter(az_i, z_q, sigma_gauss, plot=False)

            if db_scale:
                az_i = 10 * np.log10(az_i)


            if lowpass:
                az_i = scipy.signal.filtfilt(b, a, az_i)

            # Check if current azimuth has useable return
            if np.all(az_i == 0):
                u_i = np.nan
                del_r = np.nan
                del_r_pix = np.nan
                print("here")
                continue

            s_i = np.sqrt(np.sum(az_i**2))
            C_array = np.zeros((2*num_neighbours, len(az_i)*2-1))
            if upsample: C_array = np.zeros((2*num_neighbours, len(az_i)*2-1))
            del_r_array = np.zeros((2*num_neighbours, 1))
            num_jj = 0
            for jj in range(-num_neighbours, num_neighbours+1):
                # Load jj'th opposite chirp azimuth on each side and process
                if jj == 0: continue
                if jj < 0:
                    az_idx = ii-1+2*(jj+1)
                else:
                    az_idx = ii+1+2*(jj-1)
                az_i_jj = fft_data_use[az_idx, min_range_pix:max_range_pix]

                if avg_with_neighbours:
                    if jj < 0:
                        az_i_m2 = fft_data_use[az_idx-2, min_range_pix:max_range_pix]
                        az_i_jj = (az_i_jj + az_i_m2) / 2
                    else:
                        az_i_p2 = fft_data_use[az_idx+2, min_range_pix:max_range_pix]
                        az_i_jj = (az_i_jj + az_i_p2) / 2

                if use_gt_vel_shift:
                    # Instead of loaded az_i_jj, use az_i shifted by gt velocity
                    if ii % 2 == 0:
                        del_r_gt = 2*(u_gt[ii] * beta_up) / radar_res
                    else:
                        del_r_gt = 2*(u_gt[ii] * beta_down) / radar_res
                    
                    az_i_jj = np.roll(az_i, int(del_r_gt))

                # Process if necessary
                if subtract_mean: az_i_jj = az_i_jj - np.mean(az_i_jj)
                if normalize: az_i_jj = az_i_jj / np.max(az_i_jj)
                if remove_mean: az_i_jj[az_i_jj < 2*np.mean(fft_data_use[az_idx])] = 0.0
                if square_signal: az_i_jj = az_i_jj**2
                if exp_signal: az_i_jj = np.exp(az_i_jj)
                if upsample: az_i_jj = scipy.signal.resample(az_i_jj, upsample_factor*len(az_i_jj))

                if moving_average:
                    ret = np.cumsum(az_i_jj)
                    ret[ma_n:] = ret[ma_n:] - ret[:-ma_n]
                    az_i_jj = ret[ma_n - 1:] / ma_n

                if cfar_enabled:
                    cfar_jj = cfar_fft[0, az_idx, min_range_pix:max_range_pix]
                    if upsample:
                        cfar_jj = scipy.signal.resample(cfar_jj, upsample_factor*len(cfar_jj))
                    az_i_jj = az_i_jj * cfar_jj

                if cen_filtering:
                    az_i_jj = cen_filter(az_i_jj, z_q, sigma_gauss, plot=False)
                
                if db_scale:
                    az_i_jj = 10 * np.log10(az_i_jj)

                if lowpass:
                    az_i_jj = scipy.signal.filtfilt(b, a, az_i_jj)

                corr_jj_i = scipy.signal.correlate(az_i, az_i_jj, mode='full', method='fft')
                # Save normalized correlation
                s_jj = np.sqrt(np.sum(az_i_jj**2))
                if np.all(corr_jj_i == 0):
                    num_jj += 1
                    continue
                else:
                    C_jj = corr_jj_i / (s_i * s_jj)
                    # Extract pixel shift from this correlation
                    # Divide by 2 to get del_r to true range
                    del_r_jj = -(np.argmax(corr_jj_i) - (len(az_i) - 1))/2

                    if abs(del_r_jj) > 30:
                        num_jj += 1
                        continue
                if upsample: del_r_jj = del_r_jj / upsample_factor
                # Save correlation and del_r
                C_array[num_jj] = C_jj
                del_r_array[num_jj] = del_r_jj
                num_jj += 1
                

                if ii % 2 == 0:
                    del_r_jj_shift_val = del_r_jj*upsample_factor
                    del_r_gt = (u_gt[ii] * beta_up) / radar_res * upsample_factor
                else:
                    del_r_jj_shift_val = -del_r_jj*upsample_factor
                    del_r_gt = -(u_gt[ii] * beta_down) / radar_res * upsample_factor



                # Fit Gaussian to correlation result
                mean_idx = np.argmax(corr_jj_i)
                indices = np.arange(len(corr_jj_i))

                use_idx_start = mean_idx - var_window
                use_idx_end = mean_idx + var_window

                indices = indices[use_idx_start:use_idx_end]

                # Repeat indices by the value of the correlation
                #indices = np.repeat(indices, np.round(corr_jj_i).astype(int))
                # Compute mean and std
                #std = np.std(indices)

                corr_jj_i_norm = corr_jj_i[use_idx_start: use_idx_end] / np.sum(corr_jj_i[use_idx_start: use_idx_end])

                var = np.sum(corr_jj_i_norm * (indices - mean_idx)**2)
                err = del_r_jj_shift_val - del_r_gt
                del_r_est_list.append(del_r_jj_shift_val)
                del_r_gt_list.append(del_r_gt)
                del_r_az_list.append(azimuths[ii])
                var_u.append(var)
                err_u.append(err)
                NEES_list.append(err**2 / var)
            
                #print( del_r_jj_shift_val)
                #print( del_r_gt)



                #std = np.sqrt(var)

                
                """
                print("error: ", del_r_jj_shift_val - del_r_gt)

                #if (del_r_jj_shift_val - del_r_gt) > 0:
                #print("std: ", std)
                fig = plt.figure()
                plt.plot(corr_jj_i / np.sum(corr_jj_i))
                # Plot red line at del_r_gt and blue line at jj_shift
                plt.axvline(x=np.argmax(corr_jj_i), color='b')
                plt.axvline(x=del_r_gt + (len(az_i) - 1), color='r')
                # Plot Gaussian fit
                x = np.arange(len(corr_jj_i))
                #y = corr_jj_i[mean_idx] * np.exp( - (x - mean_idx)**2 / (2 * std**2))
                #y = 1/(std * np.sqrt(2*np.pi)) * np.exp( - (x - mean_idx)**2 / (2 * std**2))
                #plt.plot(x, y, 'k--')
                
                """
                # if ii > 30 and jj == 1:
                #     # print(ii, jj)
                #     # fig, axs = plt.subplots(3, 1)
                #     # axs[0].plot(az_i)
                #     # axs[0].plot(az_i_jj)
                #     # axs[0].set_title("Original")
                #     # axs[1].plot(az_i)
                #     # az_i_jj_roll_est = np.roll(az_i_jj, int(del_r_jj_shift_val))
                #     # axs[1].plot(az_i_jj_roll_est)
                #     # axs[1].set_title("Estimated")
                #     # axs[2].plot(az_i)
                #     # az_i_jj_roll_gt = np.roll(az_i_jj, int(del_r_gt))
                #     # axs[2].plot(az_i_jj_roll_gt)
                #     # axs[2].set_title("Ground Truth")
                #     # plt.show()


                #     fig = plt.figure(figsize=(6, 2))
                #     plt.plot(az_i_og[:1500], label='$\mathrm{z}_i$')
                #     plt.plot(az_i_jj_og[:1500], label='$\mathrm{z}_{i+1}$')
                #     plt.xlabel('Range bin')
                #     plt.ylabel('Amplitude')
                #     plt.legend()
                #     plt.tight_layout()

                #     fig = plt.figure(figsize=(6,2))
                #     plt.plot(az_i[:1500], label='$\mathrm{z}_i$')
                #     plt.plot(az_i_jj[:1500], label='$\mathrm{z}_{i+1}$')
                #     plt.xlabel('Range bin')
                #     plt.ylabel('Amplitude')
                #     plt.legend()
                #     plt.tight_layout()

                #     az_i_jj_roll_est = np.roll(az_i_jj, int(del_r_jj_shift_val))


                #     fig = plt.figure(figsize=(6, 2))
                #     plt.plot(az_i[:1500], label='$\mathrm{z}_i$')
                #     plt.plot(az_i_jj_roll_est[:1500], label='$\mathrm{z}_{i+1}$')
                #     plt.xlabel('Range bin')
                #     plt.ylabel('Amplitude')
                #     plt.legend()
                #     plt.tight_layout()

                #     fig = plt.figure(figsize=(6, 3))


                #     plt.show()
                #     # fdas
                


            # Compute how different each del_r is from the others
            del_r_diff = np.zeros_like(del_r_array)
            for jj in range(C_array.shape[0]):
                del_r_diff[jj] = np.abs(np.mean(del_r_array[~jj]) - del_r_array[jj])

            # Process correlations
            corr_avg = np.ones(C_array.shape[1])
            M = 0
            for jj in range(C_array.shape[0]):
                #if jj > 0: continue
                if del_r_diff[jj] > max_del_r_diff or np.all(C_array[jj] == 0):
                    continue
                
                # Compute range correction
                corr_avg *= (1 - C_array[jj]**2)
                M += 1
            if M < 2:
                u_i = np.nan
                del_r = np.nan
                del_r_pix = np.nan
                u_sig = np.nan
            else:
                if join_corr:
                    corr_avg = 1 - corr_avg**(1/M)
                    del_r_pix = -(np.argmax(corr_avg) - (len(az_i) - 1))/2
                    if upsample: del_r_pix = del_r_pix / upsample_factor                

                    # Compute standar deviation in corr_avg
                    # First, get second derivative of corr_avg
                    corr_avg_2 = np.gradient(np.gradient(corr_avg))
                    corr_sig = np.sqrt(1/(- M * corr_avg.shape[0] * corr_avg_2 * np.sqrt(corr_avg) / (1 - corr_avg)))
                    # Pick off covariance from max of corr_avg
                    u_sig = corr_sig[np.argmax(corr_avg)]
                else:
                    # Compute del_r based on average of del_r's
                    #del_r_pix = np.mean(del_r_array[del_r_diff <= max_del_r_diff])
                    del_r_pix = np.max(del_r_array[del_r_diff <= max_del_r_diff])
                    u_sig = np.nan

                del_r = del_r_pix * radar_res

                # Shift fft_range_corr by del_r_pix
                fft_range_corr[ii, :] = np.roll(fft_range_corr[ii, :], int(del_r_pix))

                # Find azimuth velocity
                if ii % 2 == 0:
                    u_i = del_r / beta_up
                else:
                    u_i = del_r / beta_down

            u_scan[ii, :] = u_i
            u_az[ii] = u_i
            u_az_gt[ii] = u_gt[ii]
            del_r_gt = (u_gt[ii] * beta_up) / radar_res
            #print("Azimuth: {}, del_r_m1: {}, del_r_p1: {}, del_r_gt: {}".format(ii, del_r_m1, del_r_p1, del_r_gt))
            #print("Azimuth: {}, del_r_pix: {}, del_r_gt: {}, u: {}, u_gt: {}".format(ii, del_r_pix, del_r_gt, u_i, u_gt[ii]))
            print("Azimuth {}, u: {}, u_gt: {}, u_err: {}, u_sig: {}".format(ii, u_i, u_gt[ii], u_i - u_gt[ii], u_sig))

            if plot:
                axs[idx].plot(az_i)
                axs[idx].plot(fft_data[ii, min_range_pix:max_range_pix])
                axs[idx].set_title("Azimuth: {}, del_r: {}, u: {}".format(ii, del_r, u_i))
                plt.subplots_adjust(hspace=0.5)
    #plt.show()

    NEES = np.mean(NEES_list)
    print("NEES: ", NEES)

    # Fit cosine curve to del_r_est_list
    A = np.cos(del_r_az_list)[:, np.newaxis]
    b = np.array(del_r_est_list)

    #model = linear_model.LinearRegression(fit_intercept=False)
    #ransac = linear_model.RANSACRegressor(estimator=model, min_samples=2, residual_threshold=5.0, max_trials=100, stop_probability=0.99999)
    #ransac.fit(A, b)
    #inlier_mask = ransac.inlier_mask_

    max_ransac_iter = 50
    ransac_threshold = 0.8
    best_num_inlier = 0
    for iter in range(max_ransac_iter):
        # Randomly sample 2 points
        idx = np.random.choice(A.shape[0], 10, replace=False)
        A_sample = A[idx]
        b_sample = b[idx]
        # Fit line to sample
        model = linear_model.LinearRegression(fit_intercept=False)
        model.fit(A_sample, b_sample)
        # Compute residuals
        residuals = A @ model.coef_ - b
        # Compute inliers
        num_inlier = np.sum(np.abs(residuals) < ransac_threshold)
        if num_inlier > best_num_inlier:
            best_num_inlier = num_inlier
            best_model = model
            best_residuals = residuals
            best_inlier_mask = np.abs(residuals) < ransac_threshold

    inlier_mask = best_inlier_mask
    A = A[inlier_mask]
    b = b[inlier_mask]
    del_r_est = scipy.optimize.least_squares(vel_est_err, 0.0, args=(A, b), loss='cauchy', f_scale=5.0)

    print(del_r_est.x)
    print(np.min(del_r_gt_list))

    print(del_r_est.x * radar_res / beta_up)

    # Plot error and variance
    fig = plt.figure()
    plt.subplot(1, 2, 1)
    plt.plot(err_u, 'ko')
    plt.plot(np.sqrt(var_u), 'r--')
    plt.plot(-np.sqrt(var_u), 'r--')
    plt.legend(['Error', 'std'])
    plt.title("Error and std, NEES: {}".format(NEES))

    plt.subplot(1, 2, 2)
    plt.plot(del_r_est_list, 'ko')
    plt.plot(del_r_gt_list, 'r--')
    plt.plot(del_r_est.x * np.cos(del_r_az_list), 'g--')
    plt.legend(['Estimated', 'Ground Truth'])

    
    #plt.show()

    #sdfads

    # Plot fft_data and fft_range_corr side by side
    cart_range_corr = radar_polar_to_cartesian(fft_range_corr, azimuths, radar_res, interpolate_crossover=True, fix_wobble=True)
    cart_og = radar_polar_to_cartesian(fft_data, azimuths, radar_res, interpolate_crossover=True, fix_wobble=True)
    fig = plt.figure()
    plt.subplot(1, 2, 1)
    plt.imshow(cart_og, cmap='gray')
    plt.title("Original")
    plt.subplot(1, 2, 2)
    plt.imshow(cart_range_corr, cmap='gray')
    plt.title("Range-corrected")

    # Run RANSAC to find inlier u_az values
    az_use = azimuths[np.isnan(u_az) == False]
    u_az_use = u_az[np.isnan(u_az) == False]
    b = u_az_use.copy()
    #A = np.hstack((np.cos(az_use)[:, np.newaxis], np.sin(az_use)[:, np.newaxis]))
    A = np.cos(az_use)[:, np.newaxis]

    # First downsample based on RANSAC
    #model = linear_model.LinearRegression(fit_intercept=False)
    #ransac = linear_model.RANSACRegressor(estimator=model, min_samples=2, residual_threshold=0.45, max_trials=100, stop_probability=0.99999)
    #ransac.fit(A, b)
    #inlier_mask = ransac.inlier_mask_
    #residuals = A[inlier_mask] @ ransac.estimator_.coef_.reshape(-1,1) - b[inlier_mask].reshape(-1,1)

    max_ransac_iter = 50
    ransac_threshold = 0.8
    best_num_inlier = 0
    for iter in range(max_ransac_iter):
        # Randomly sample 2 points
        idx = np.random.choice(A.shape[0], 10, replace=False)
        A_sample = A[idx]
        b_sample = b[idx]
        # Fit line to sample
        model = linear_model.LinearRegression(fit_intercept=False)
        model.fit(A_sample, b_sample)
        # Compute residuals
        residuals = A @ model.coef_ - b
        # Compute inliers
        num_inlier = np.sum(np.abs(residuals) < ransac_threshold)
        if num_inlier > best_num_inlier:
            best_num_inlier = num_inlier
            best_model = model
            best_residuals = residuals
            best_inlier_mask = np.abs(residuals) < ransac_threshold

    print(best_num_inlier)
    inlier_mask = best_inlier_mask


    fig = plt.figure()
    plt.plot(A[:,0], b, 'o', label='Original')
    plt.plot(A[:,0], A[:,0] * vel_gt[gt_idx, 0], label='Ground Truth')
    plt.plot(A[inlier_mask, 0], b[inlier_mask], 'ro', label='Inliers')
    plt.plot(A[:, 0], A[:, 0] * best_model.coef_[0], label='RANSAC', color='red' )
    plt.legend()
    plt.xlabel('Cosine of azimuth')
    plt.ylabel('Azimuth velocity')
    #plt.show()
    #sfda

    fig = plt.figure(figsize=(6,4))
    plt.plot(A[:,0], b, 'o', label='$\mathbf{U}$')
    plt.plot(A[inlier_mask, 0], b[inlier_mask], 'ro', label='$\mathbf{U}_R$')
    plt.legend()
    plt.xlabel('$\cos(\phi)$')
    plt.ylabel('$u$ [m/s]')


    #inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)
    u_az_use[outlier_mask] = np.nan
    u_az_ransac = u_az.copy()
    u_az_ransac[np.isnan(u_az) == False] = u_az_use
    A = A[inlier_mask]
    b = b[inlier_mask]
    vel_est = scipy.optimize.least_squares(vel_est_err, vel_gt[gt_idx, 0:1], args=(A, b), loss='cauchy', f_scale=0.6)


    print(vel_gt[gt_idx, 0:3])
    print(vel_est.x)
    print(vel_est.x.squeeze() - vel_gt[gt_idx, 0:1].squeeze())



    pc_RANSAC = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az_ransac, 0))
    pc_RANSAC = pc_RANSAC[0]

    """
    fig = plt.figure()
    plt.plot(v_vehicle_gt[:,1], 'o')
    plt.plot(v_vehicle_est_y, 'o')
    
    #plt.plot(v_vehicle_gt[:, 0], 'x')
    #plt.plot(v_vehicle_gt[:, 1], 'x')
    #plt.legend(['x', 'y'])
    plt.title("Estimated vehicle velocity Y")
    fig = plt.figure()
    plt.plot(v_vehicle_gt[:,0], 'o')
    plt.plot(v_vehicle_est_x, 'o')
    
    #plt.plot(v_vehicle_gt[:, 0], 'x')
    #plt.plot(v_vehicle_gt[:, 1], 'x')
    #plt.legend(['x', 'y'])
    plt.title("Estimated vehicle velocity X")
    plt.show()

    # Back-compute u_az from v_vehicle_est_x
    u_az[~np.isnan(u_az)] = v_vehicle_y_ransac / np.cos(azimuths_ransac)
    """

    # Find CFAR mask for fft data
    pc = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az, 0))
    pc = pc[0]
    pc_gt = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_az_gt, 0))
    pc_gt = pc_gt[0]

    # Compute error based on extracted points
    #err = pc_gt[:, 3] - pc[:, 3]
    err = pc_gt[:, 3] - pc_RANSAC[:, 3]
    RMSE = np.sqrt(np.mean(err[~np.isnan(err)]**2))
    print("RMSE: {}".format(RMSE))
    print("Mean error: {}".format(np.mean(err[~np.isnan(err)])))
    print("Abs mean error: {}".format(np.mean(np.abs(err[~np.isnan(err)]))))
    print("Max error: {}".format(np.max(np.abs(err[~np.isnan(err)]))))

    err_az = np.nan * np.zeros_like(u_gt)
    err_az[az_start:az_end+1] = u_gt[az_start:az_end+1] - u_az_ransac[az_start:az_end+1]
    pc_err = extract_pc(cfar_fft, radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(err_az, 0))
    pc_err = pc_err[0]

    # Align fft_data_marked with pointcloud xy coordinates
    radar_cart_img_marked = np.rot90(radar_cart_img_marked, k=1, axes=(0,1))
    radar_cart_img_marked = np.flip(radar_cart_img_marked, axis=0)
    fig = plt.figure()
    plt.imshow(radar_cart_img_marked)
    plt.colorbar()

    # Plot cartesian with overlaid points
    radar_cart_img = radar_polar_to_cartesian(fft_data, azimuths, radar_res,
                                              interpolate_crossover=True, fix_wobble=True)
    
    motion_type = 'dynamic'  # 'static' or 'dynamic'
    gridspec = {'width_ratios': [5, 5, 5, 0.2]}
    fig, axs = plt.subplots(1, 4, figsize=(15.2, 5.2), gridspec_kw=gridspec)
    plt.subplot(1, 4, 1)
    visualize_doppler(radar_cart_img, pc_gt, start_fig=False, show_colourbar=False)
    plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2, -15, 0, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)
    
    if motion_type == 'static': plt.title("Ground Truth", size=16)
    plt.subplot(1, 4, 2)
    visualize_doppler(radar_cart_img, pc, start_fig=False, show_colourbar=False)
    plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2, -15, 0, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)

    if motion_type == 'static':  plt.title("Raw Estimates", size=16)
    plt.subplot(1, 4, 3)
    visualize_doppler(radar_cart_img, pc_RANSAC, start_fig=False, show_colourbar=False)
    plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2, -15, 0, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)

    if motion_type == 'static': plt.title("RANSAC Estimates", size=16)
    cax=axs[3]
    plt.colorbar(cax=cax)
    if motion_type == 'static': axs[0].set_ylabel('Static', size=16)
    elif motion_type == 'dynamic': axs[0].set_ylabel('Dynamic', size=16)
    axs[3].set_ylabel('Velocity [m/s]', size=16)

    # Flip all axes so that car is going left to right
    for ax in axs:
        ax.set_xlim(ax.get_xlim()[::-1])

    plt.tight_layout()
    #
    # plt.subplot(1, 4, 4)
    # visualize_doppler(radar_cart_img, pc_err, start_fig=False, show_colourbar=True)
    # plt.title("Error")
    # Share y axis
    

    # Plot cartesian image and overlay with color-coded velocity values from u_scan
    u_scan_cart = radar_polar_to_cartesian(u_scan, azimuths, radar_res, interpolate_crossover=True, fix_wobble=True)
    # Align u_scan_cart with pointcloud xy coordinates
    u_scan_cart = np.rot90(u_scan_cart, k=1, axes=(0,1))
    u_scan_cart = np.flip(u_scan_cart, axis=0)
    fig = plt.figure()
    plt.imshow(u_scan_cart, cmap='viridis')
    plt.colorbar()
    

    fig = plt.figure()
    plt.hist(err[~np.isnan(err)], bins=100)
    plt.title("Error histogram")
    plt.show()


def vel_est_err(x, A, b):
    return b - A @ x


if __name__ == "__main__":
    main()