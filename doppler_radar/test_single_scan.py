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


def get_avg_pts(log_file):
    point_counts = []
    with open(log_file) as infile:
        for line in infile:
            if "Extracted " in line:
                point_counts.append(int(line.split(" ")[6]))
    avg_pts = sum(point_counts) / len(point_counts)
    return int(avg_pts)

def get_ros_gps_times(path='camera_event_time.csv'):
    ros_times = []
    gps_times = []
    with open(path, 'r') as f:
        f.readline()
        for line in f:
            parts = line.split(',')
            ros_times.append(strToTime(parts[0]))
            gps_times.append(float(parts[1]))
    return ros_times, gps_times

def strToTime(tin):
    tstr = str(tin)
    if '.' in tstr:
        return float(tstr)
    t = float(tstr)
    timeconvert = 1e-6
    if len(tstr) != 16 and len(tstr) > 10:
        timeconvert = 10**(-1 * (len(tstr) - 10))
    return t * timeconvert

def main():
    # Seq for paper 1 plot
    # seq = "boreas-2024-11-02-14-44"
    # frame = '1730573088018978'

    seq = "boreas-2024-11-12-13-37"
    frame = '1731436633523655'

    seq = 'boreas-2024-11-26-16-17'
    frame = '1732655862202206'

    # seq = "boreas-2024-11-04-14-32"
    # frame = '1730747590633483'

    # seq = "boreas-2024-01-09-14-00"
    # frame = '1704826804513623'

    seq = "boreas-2024-11-02-14-14"
    frame = '1730571248599238'

    seq = "boreas-2024-11-27-13-50"
    frame = '1732733459158859'

    # seq = 'boreas-2024-11-27-14-09'
    # frame = '1732734577981861'

    # seq = 'boreas-2024-11-28-13-46'
    # frame = '1732819619376723'



    # Load in all config parameters
    file_path = 'config/doppler_config.yaml'
    extractor = DopplerExtractor(file_path)

    # Load in Boreas data and other data paths
    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    result_dir = osp.join(data_dir, 'vtr_results')
    seq_path = osp.join(dataset_dir, seq)
    rad_seq_path = osp.join(seq_path, 'radar')

    bd = BoreasDataset(dataset_dir, split = None, verbose=False)
    bd_seq = bd.get_seq_from_ID(seq)

    vel_gt = np.asarray([frame.body_rate for frame in bd_seq.radar_frames]).squeeze(-1)
    time_gt = np.asarray([frame.timestamp_micro for frame in bd_seq.radar_frames])
    all_plot_data = []
    img_path = osp.join(rad_seq_path, frame + '.png')
    gt_idx = np.where(time_gt == int(frame))[0][0]
    
    # Load in raw radar data
    radar_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    fft_data, azimuths, timestamps, up_chirps = load_radar(radar_img)
    fft_data[:, :42] = 0    # Zero out first ~2m to get rid of reflections from our car


    ### DELETE BELOW
    # Analyze timestamps
    print("Timestamp span: ", (timestamps[-1] - timestamps[0])*1e-6)
    print("Timestamp diff: ", np.mean(np.diff(timestamps))*1e-6)

    print("Frame rate: ", (time_gt[-1] - time_gt[0])*1e-6 / len(time_gt))
    print("Max gap: ", np.max(np.diff(time_gt))*1e-6)

    applanix_dir = osp.join(seq_path, 'applanix')
    # IMU file has fields GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx
    imu_raw_file = osp.join(applanix_dir, 'imu_raw.csv')
    imu_raw_data = np.loadtxt(imu_raw_file, delimiter=',', skiprows=1)
    imu_raw_time = imu_raw_data[:, 0]
    imu_raw_gyro = np.stack((imu_raw_data[:, 3], imu_raw_data[:, 2], imu_raw_data[:, 1]), axis=1)
    imu_raw_accel_norm = imu_raw_data[:, 6]
    imu_raw_yaw = imu_raw_gyro[:, 2] * 180 / np.pi

    #IMU file has fields time,qx,qy,qz,qw,wx,wy,wz,ax,ay,az
    imu_dmu_file = osp.join(applanix_dir, 'dmu_imu.csv')
    imu_dmu_data = np.genfromtxt(imu_dmu_file, delimiter=',', skip_header=1, dtype=[('time', 'i8'), ('raw_count', 'f16'), ('msg_num', 'f16'), ('qx', 'f16'), ('qy', 'f16'), ('qz', 'f16'), ('qw', 'f16'), ('wx', 'f16'), ('wy', 'f16'), ('wz', 'f16'), ('ax', 'f16'), ('ay', 'f16'), ('az', 'f16')])

    # Extract columns
    imu_dmu_time = imu_dmu_data['time']
    imu_dmu_raw_count = imu_dmu_data['raw_count']
    imu_dmu_msg_num = imu_dmu_data['msg_num']
    imu_dmu_accel_norm = imu_dmu_data['ay']
    imu_dmu_gyro = np.stack((imu_dmu_data['wx'], imu_dmu_data['wy'], imu_dmu_data['wz']), axis=1)
    imu_dmu_yaw = imu_dmu_gyro[:, 2] * 180 / np.pi

    # Filter out data where the change in msg count is randomly huge
    imu_dmu_msg_diff = np.diff(imu_dmu_msg_num)
    dmu_data_keep_idx = np.ones(imu_dmu_time.shape[0], dtype=bool)
    dmu_data_keep_idx[1:] = np.abs(imu_dmu_msg_diff) < 100
    imu_dmu_time = imu_dmu_time[dmu_data_keep_idx]
    imu_dmu_raw_count = imu_dmu_raw_count[dmu_data_keep_idx]
    imu_dmu_msg_num = imu_dmu_msg_num[dmu_data_keep_idx]
    imu_dmu_gyro = imu_dmu_gyro[dmu_data_keep_idx]
    imu_dmu_yaw = imu_dmu_yaw[dmu_data_keep_idx]
    imu_dmu_accel_norm = imu_dmu_accel_norm[dmu_data_keep_idx]
    


    #Aeva IMU file has fields time,wx,wy,wz,ax,ay,az
    imu_aeva_file  = osp.join(applanix_dir, 'aeva_imu.csv')
    imu_aeva_data = np.loadtxt(imu_aeva_file, delimiter=',', skiprows=1)
    imu_aeva_time = imu_aeva_data[:, 0]
    imu_aeva_gyro = np.stack((imu_aeva_data[:, 1], imu_aeva_data[:, 2], imu_aeva_data[:, 3]), axis=1)
    imu_aeva_yaw = imu_aeva_gyro[:, 2] * 180 / np.pi

    time_gt_proc = time_gt/1e6# - time_gt[0]/1e6
    imu_raw_time_proc = imu_raw_time - time_gt[0]/1e6
    imu_dmu_time_proc = imu_dmu_time/1e9 - time_gt[0]/1e6
    imu_aeva_time_proc = imu_aeva_time/1e6 - time_gt[0]/1e6
    time_gt_proc = time_gt/1e6 - time_gt[0]/1e6


    print(imu_dmu_time[0])
    print(imu_dmu_time[0]/1e9)
    print(time_gt[0]/1e6)

    ros_times, gps_times = get_ros_gps_times(osp.join(applanix_dir, 'ros_and_gps_time.csv'))
    ros_times = np.asarray(ros_times).reshape(-1, 1)
    gps_times = np.asarray(gps_times).reshape(-1, 1)
    deltas = ros_times - gps_times
    delta = np.abs(np.mean(deltas))

    print("Max offset between ros and gps: ", max(deltas)[0])
    print("Mean offset between ros and gps: ", delta)

    log_path = osp.join(result_dir, 'radar', seq, seq)
    list_log_files = os.listdir(log_path)
    list_log_files = [f for f in list_log_files if f.endswith('.log')]
    list_log_files.sort()
    avg_pts = get_avg_pts(osp.join(log_path, list_log_files[-1]))
                          
    print("Average points: ", avg_pts)

    # IMU timing processing
    imu_dmu_time_s = imu_dmu_time_proc
    dmu_freq = imu_dmu_time_s.shape[0] / (imu_dmu_time_s[-1] - imu_dmu_time_s[0])
    dmu_max_gap = np.max(np.diff(imu_dmu_time_s))
    dmu_mean_gap = np.mean(np.diff(imu_dmu_time_s))
    dmu_min_gap = np.min(np.diff(imu_dmu_time_s))
    print("DMU freq: ", dmu_freq)
    print("DMU max gap: ", dmu_max_gap)
    print("DMU mean gap: ", dmu_mean_gap)
    print("DMU min gap: ", dmu_min_gap)

    # Resample DMU according to msg count
    #imu_dmu_time_resampled = imu_dmu_time_s[0] + (imu_dmu_msg_num-imu_dmu_msg_num[0]) * 1/200
    imu_dmu_time_resampled = imu_dmu_time_s[0] + (imu_dmu_msg_num-imu_dmu_msg_num[0]) * (imu_dmu_time_s[-1] - imu_dmu_time_s[0]) / (imu_dmu_msg_num[-1] - imu_dmu_msg_num[0])

    dmu_idx_select = np.where((imu_dmu_time_resampled > 4) & (imu_dmu_time_resampled < 14))
    gt_idx_select = np.where((imu_raw_time_proc > 4) & (imu_raw_time_proc < 14))
    cc_dmu_time = imu_dmu_time_resampled[dmu_idx_select]
    cc_dmu_accel_norm = imu_dmu_accel_norm[dmu_idx_select]
    cc_gt_time = imu_raw_time_proc[gt_idx_select]
    cc_gt_accel_norm = imu_raw_accel_norm[gt_idx_select]

    cc_result = scipy.signal.correlate(cc_dmu_accel_norm, cc_gt_accel_norm, mode='full', method='fft')
    pix_shift = -(np.argmax(cc_result) - (len(cc_dmu_time) - 1))
    t_shift = pix_shift * (cc_dmu_time[-1] - cc_dmu_time[0]) / len(cc_dmu_time)

    print("Pixel shift: ", pix_shift)
    print("Time shift: ", t_shift)

    #imu_dmu_time_proc = imu_dmu_time_proc + t_shift
    imu_dmu_time_resampled = imu_dmu_time_resampled + t_shift
    imu_dmu_time_resampled_save = (imu_dmu_time_resampled + time_gt[0]/1e6) * 1e9

    print(imu_dmu_time_resampled_save[0])

    # Save new DMU results to new csv
    # new_dmu_file = osp.join(applanix_dir, 'dmu_imu_shifted.csv')
    # new_dmu_data = np.stack((imu_dmu_time_resampled_save, imu_dmu_data['wx'], imu_dmu_data['wy'], imu_dmu_data['wz'], imu_dmu_data['ax'], imu_dmu_data['ay'], imu_dmu_data['az']), axis=1)
    # np.savetxt(new_dmu_file, new_dmu_data, delimiter=',', header='time,wx,wy,wz,ax,ay,az', comments='')
    # print("Saved new DMU file to: ", new_dmu_file)

    # fixed_dmu_time = cc_dmu_time + t_shift
    # fig = plt.figure()
    # plt.plot(cc_gt_time, cc_gt_accel_norm, label='Groundtruth')
    # plt.plot(cc_dmu_time, cc_dmu_accel_norm, label='DMU')
    # plt.plot(fixed_dmu_time, cc_dmu_accel_norm, label='Fixed DMU gyro')
    # plt.xlabel('Time [s]', size=24)
    # plt.xticks(size=20)
    # plt.ylabel('Fwd Accel [m/s\^2)]', size=24)
    # plt.yticks(size=20)
    # plt.legend(fontsize=20)
    # plt.show()
    # fdsa
    

    

    fig = plt.figure()
    # plt.plot(imu_dmu_time_proc, imu_dmu_yaw, label='DMU gyro', color='C1')
    plt.plot(imu_aeva_time_proc, -imu_aeva_yaw, label='Aeva gyro', color='C2')
    plt.plot(imu_raw_time_proc, imu_raw_yaw, label='Raw gyro', color='C0')
    plt.plot(time_gt_proc, vel_gt[:, 5]*180/np.pi, label='Groundtruth', color='C3')
    # plt.plot(imu_dmu_time_resampled, imu_dmu_yaw, label='DMU resampled', color='C4')

    plt.xlabel('Time [s]', size=24)
    plt.xticks(size=20)
    plt.ylabel('Yaw [deg/s]', size=24)
    plt.yticks(size=20)
    plt.legend(fontsize=20) 
    #plt.show()



    fig = plt.figure()
    plt.plot(imu_dmu_time_s[:2000], np.diff(imu_dmu_time_s[:2001]), label='DMU', color='C1')
    plt.plot(imu_aeva_time_proc[:1000], np.diff(imu_aeva_time_proc[:1001]), label='Aeva', color='C2')
    plt.plot(imu_raw_time_proc[:2000] - imu_raw_time_proc[0], np.diff(imu_raw_time_proc[:2001]), label='Raw', color='C0')
    plt.plot(imu_dmu_time_resampled[:2000], np.diff(imu_dmu_time_resampled[:2001]), label='DMU resampled', color='C4')
    plt.xlabel('Time [s]', size=24)
    plt.xticks(size=20)
    plt.ylabel('Boreas Measurement Time Gap [s]', size=24)
    plt.yticks(size=20)
    plt.legend(fontsize=20)

    fig = plt.figure()
    plt.plot(imu_dmu_time_s[:-1], np.diff(imu_dmu_msg_num), label='DMU', color='C1')
    plt.xlabel('Time [s]', size=24)
    plt.xticks(size=20)
    plt.ylabel('DMU Message Gap', size=24)
    plt.yticks(size=20)
    plt.legend(fontsize=20)

    fig = plt.figure()
    plt.plot(imu_dmu_time_s, imu_dmu_msg_num, label='DMU', color='C1')
    plt.xlabel('Time [s]', size=24)
    plt.xticks(size=20)
    plt.ylabel('DMU Message Number', size=24)
    plt.yticks(size=20)
    plt.legend(fontsize=20)

    fig = plt.figure()
    plt.plot(imu_dmu_time_s)
    plt.xlabel('Count', size=24)
    plt.xticks(size=20)
    plt.ylabel('DMU Time [s]', size=24)
    plt.yticks(size=20)

    fig = plt.figure()
    plt.plot(imu_dmu_time_s, imu_dmu_raw_count, label='DMU', color='C1')
    plt.xlabel('Time [s]', size=24)
    plt.xticks(size=20)
    plt.ylabel('DMU Raw Count', size=24)
    plt.yticks(size=20)
    plt.legend(fontsize=20)
    

    
    # plt.legend()
    plt.show()
    fdsafdsa

    # fig = plt.figure()
    # plt.plot(vel_gt[:,0], label='fwd')
    # plt.plot(vel_gt[:,1], label='side')
    # plt.show()


    afds

    ### DELETE ABOVE

    # Extract vehicle velocity gt
    v_gt = vel_gt[gt_idx, 0:2]

    # Estimate vehicle velocity
    # Either directly
    vel_est, u_est, az_est = extractor.extract_vehicle_velocity(fft_data, azimuths, prior_vel=v_gt, return_u=True)

    # Or part-by-part
    u_raw, az_raw = extractor.extract_doppler(fft_data, azimuths)
    u_ransac, az_ransac = extractor.ransac(u_raw, az_raw)
    vel_est = extractor.estimate_vel_from_radial(u_ransac, az_ransac, v_init=vel_gt[gt_idx, 0:2])

    # Get gt azimuth-wise velocities
    u_gt = v_gt[0] * np.cos(az_est) + v_gt[1] * np.sin(az_est)

    percent_0 = np.sum(np.abs(u_est[~np.isnan(u_est)]) < 0.5) / len(u_est[~np.isnan(u_est)])
    print("Percent of 0 values: ", percent_0)

    # Compute azimuth-wise RMSE
    az_rmse = extractor.compute_RMSE(u_est, az_est, v_gt)

    print("azimuth RMSE: ", az_rmse)
    print("Gt vel: ", vel_gt[gt_idx, 0:3])
    print("Est fwd: ", vel_est)
    print("Err fwd: ", vel_est.squeeze() - vel_gt[gt_idx, 0:2].squeeze())

    # Get CFAR data to use for pointcloud overlay
    cfar_fft_up = cfar_mask(np.expand_dims(fft_data, 0), extractor.radar_res, a_thresh=0.8, b_thresh=0.23, maxr=extractor.max_range)
    cfar_fft_down = cfar_mask(np.expand_dims(fft_data, 0), extractor.radar_res, a_thresh=0.8, b_thresh=0.23, maxr=extractor.max_range)
    cfar_fft = np.zeros((1, fft_data.shape[0], fft_data.shape[1]))
    cfar_fft[0, ::2, :] = cfar_fft_up[0, ::2, :]
    cfar_fft[0, 1::2, :] = cfar_fft_down[0, 1::2, :]

    # Extract groundtruth overlayed pointcloud
    pc_gt = extract_pc(cfar_fft, extractor.radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_gt, 0))
    pc_gt = pc_gt[0]
    # Extract raw extracted velocity pointcloud
    pc = extract_pc(cfar_fft, extractor.radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_raw, 0))
    pc = pc[0]
    # Extract RANSAC pointcloud
    pc_RANSAC = extract_pc(cfar_fft, extractor.radar_res, np.expand_dims(azimuths, 0), np.expand_dims(timestamps, 0), np.expand_dims(u_ransac, 0))
    pc_RANSAC = pc_RANSAC[0]

    # Plot RANSAC results
    b = u_raw[~np.isnan(u_raw)]
    A = np.hstack((np.cos(az_est[~np.isnan(u_raw)])[:, np.newaxis], np.sin(az_est[~np.isnan(u_raw)])[:, np.newaxis]))
    b_ransac = u_ransac[~np.isnan(u_ransac)]
    A_ransac = np.hstack((np.cos(az_ransac[~np.isnan(u_ransac)])[:, np.newaxis], np.sin(az_ransac[~np.isnan(u_ransac)])[:, np.newaxis]))

    fig = plt.figure(figsize=(6,4))
    plt.plot(A[:,0], b, 'ro', label=r'$\mathbf{u}$')
    plt.plot(A_ransac[:, 0], b_ransac, 'o', c='C0', label=r'$\mathbf{u}_R$')
    plt.legend(fontsize=18)
    plt.xlabel(r'$\cos(\phi)$', size=18)
    plt.ylabel(r'$u$ [m/s]', size=18)
    plt.tight_layout()

    # Save cartesian with overlaid points
    radar_cart_img = radar_polar_to_cartesian(fft_data, azimuths, extractor.radar_res,
                                            interpolate_crossover=True, fix_wobble=True)
    
    plt_data = {'timestamp': frame, 'radar_cart_img': radar_cart_img, 'pc_gt': pc_gt, 'pc': pc, 'pc_RANSAC': pc_RANSAC}

    plot_vel_overlay(plt_data)

    plt.show()

def plot_vel_overlay(plt_data):

    matplotlib.rcParams['axes.linewidth'] = 5.0
    matplotlib.rcParams['axes.edgecolor'] = 'black'

    # Plot velocity overlays
    gridspec_width = {'width_ratios': [5]}
    num_data_points = 1
    num_row = 3
    fig, axs = plt.subplots(num_row, num_data_points, figsize=(num_data_points*5.0, 15), gridspec_kw=gridspec_width)

    timestamp = plt_data['timestamp']
    radar_cart_img = plt_data['radar_cart_img']
    pc_gt = plt_data['pc_gt']
    pc = plt_data['pc']
    pc_RANSAC = plt_data['pc_RANSAC']

    # Rotate everything to point upwards
    radar_cart_img = np.rot90(radar_cart_img, 3)
    for pc_mod in [pc_gt, pc, pc_RANSAC]:
        pc_x = pc_mod[:, 0].copy()
        pc_y = pc_mod[:, 1].copy()
        pc_mod[:, 0] = -pc_y
        pc_mod[:, 1] = pc_x

    sub1 = plt.subplot(num_row, num_data_points, 1)
    visualize_doppler(radar_cart_img, pc_gt, start_fig=False, show_colourbar=True)
    plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2-7.5, 0, 15, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)
    plt.ylabel(r"Groundtruth", size=18, fontdict={'family': 'serif'})

    sub2 = plt.subplot(num_row, num_data_points, num_data_points + 1)
    visualize_doppler(radar_cart_img, pc, start_fig=False, show_colourbar=True)
    plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2-7.5, 0, 15, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)
    plt.ylabel(r"Raw Estimates", size=18, fontdict={'family': 'serif'})

    sub3 = plt.subplot(num_row, num_data_points, 2*num_data_points + 1)
    img = visualize_doppler(radar_cart_img, pc_RANSAC, start_fig=False, show_colourbar=True)
    plt.arrow(radar_cart_img.shape[0]/2, radar_cart_img.shape[1]/2-7.5, 0, 15, fc='lime', ec='k', width=12.5, head_width=25, head_length=20)
    plt.ylabel(r"RANSAC Estimates", size=18, fontdict={'family': 'serif'})

if __name__ == "__main__":
    main()