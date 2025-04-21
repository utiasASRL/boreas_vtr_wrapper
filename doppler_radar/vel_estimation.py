import os
import os.path as osp
from doppler_extractor import DopplerExtractor
import numpy as np
import matplotlib.pyplot as plt
from pyboreas import BoreasDataset
from pylgmath import se3op
import csv
import argparse
import pandas as pd
import scipy
import warnings
from eval.process_vel_estimation import eval_stats

def main(seq):
    # Load Doppler extractor
    file_path = 'config/doppler_config.yaml'
    extractor = DopplerExtractor(file_path)

    # Tuning parameters for alignment
    use_gt_yaw = False
    kf_filter = False

    # Output dirs
    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    seq_path = osp.join(dataset_dir, seq[0])
    output_dir = osp.join('results', seq[0])
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(osp.join(output_dir, 'odometry_result'), exist_ok=True)
    os.makedirs(osp.join(output_dir, 'odometry_vel_result'), exist_ok=True)
    seq_metadata_dir = osp.join(output_dir, 'per_azimuth_result')
    os.makedirs(seq_metadata_dir, exist_ok=True)
    seq_metadata_path = osp.join(seq_metadata_dir, 'metadata.csv')

    # Delete metadata file
    if osp.exists(seq_metadata_path): os.remove(seq_metadata_path)

    # Load data
    rad_seq_path = osp.join(seq_path, 'doppler_radar')
    if not osp.exists(rad_seq_path):
        print("No doppler radar data found for sequence: ", seq[0])
        return

    bd = BoreasDataset(dataset_dir, split = None, verbose=False)
    bd_seq = bd.get_seq_from_ID(seq[0])
    # Load in IMU data
    applanix_dir = osp.join(seq_path, 'applanix')
    imu_file_name = 'dmu_imu.csv'
    if not osp.exists(osp.join(applanix_dir, imu_file_name)): 
        print("Loading raw IMU data")
        imu_file_name = 'imu_raw.csv'

    if imu_file_name == 'imu_raw.csv':
        # IMU file has fields GPSTime,angvel_z,angvel_y,angvel_x,accelz,accely,accelx
        imu_file = osp.join(applanix_dir, 'imu_raw.csv')
        imu_data = np.loadtxt(imu_file, delimiter=',', skiprows=1)
        imu_time = imu_data[:, 0]
        imu_gyro = np.stack((imu_data[:, 3], imu_data[:, 2], imu_data[:, 1]), axis=1)

        # Transform imu data to radar frame
        T_applanix_radar = bd_seq.calib.T_applanix_lidar @ np.linalg.inv(bd_seq.calib.T_radar_lidar)
        imu_gyro = imu_gyro @ T_applanix_radar[:3, :3]
        # Minus needed, I assume bc the frames are inverted
        imu_yaw = -imu_gyro[:, 2]

    elif imu_file_name == 'dmu_imu.csv':
        # IMU file has fields time,qx,qy,qz,qw,wx,wy,wz,ax,ay,az
        imu_file = osp.join(applanix_dir, 'dmu_imu.csv')
        imu_data = np.loadtxt(imu_file, delimiter=',', skiprows=1)
        imu_time = imu_data[:, 0]
        imu_gyro = np.stack((imu_data[:, 5], imu_data[:, 6], imu_data[:, 7]), axis=1)
        imu_yaw = imu_gyro[:, 2]

    err_list = []
    err_y_list = []
    omega_gt_list = []
    omega_list = []
    vel_list = []
    vel_y_list = []
    est_vel_list = []
    est_vel_y_list = []
    est_vel_og_list = []
    est_vel_y_og_list = []
    timestamps_list = []
    result_vel = []

    if kf_filter:
        v_hat = np.zeros(2)
        P_hat = np.array([[0.1, 0], [0, 0.1]])
        w_a = np.array([[0.1, 0], [0, 0.1]])
        w_y = np.array([[0.2, 0], [0, 0.05]])

    T_init = None
    vel_est = np.zeros(2)
    prev_time = 0
    for rad_num, rad_frame in enumerate(bd_seq.get_radar_iter()):
        print("Change in time: ", (rad_frame.timestamp_micro - prev_time)/1e6)
        prev_time = rad_frame.timestamp_micro

        # Print progress every 500 frames
        if rad_num % 500 == 0:
            print(rad_num)

        # Load in radar data
        timestamps = rad_frame.timestamps.squeeze(1)

        # Save initial pose
        if T_init is None:
            T_init = rad_frame.pose

        fft_data = rad_frame.polar
        azimuths = rad_frame.azimuths.squeeze(1)
        vel_gt = rad_frame.body_rate

        # Unload frame data to clear up RAM, we already saved all we need
        #rad_frame.visualize(show=False)
        rad_frame.unload_data()
        
        if kf_filter:
            prior_vel = v_hat
        else:
            prior_vel = vel_est.squeeze()
        
        # Estimate vehicle velocity
        vel_est, u_est, az_est = extractor.extract_vehicle_velocity(fft_data, azimuths, prior_vel=prior_vel, return_u=True)
        vel_est_og = vel_est.copy()

        # Check if up/down chirp order has flipped
        if (vel_est[0] < 0 and vel_gt[0] > 0) or (vel_est[0] > 0 and vel_gt[0] < 0):
            vel_est[0:2] = -vel_est[0:2]
            u_est = -u_est

        if kf_filter:
            # Kalman filter
            # Prediction
            v_hat = v_hat
            P_hat = P_hat + w_a**2
            # Update
            #K = P_hat / (P_hat + cov_est[0])
            #K = P_hat @ np.linalg.inv(P_hat + w_y**2)
            K = P_hat @ np.linalg.inv(P_hat + cov_est)
            y_meas = np.array([vel_est[0], vel_est[1]]).reshape(-1,1)
            v_hat = v_hat + (K @ (y_meas - v_hat.reshape(-1,1))).squeeze()
            P_hat = (np.eye(2) - K) @ P_hat

            vel_est[0:2] = v_hat
        
        print(rad_num)
        print("Timestamp: ", rad_frame.timestamp_micro)
        print("Est vel: ", vel_est)
        print("Gt vel: ", vel_gt[:2].squeeze())
        print("Error vel: ", vel_est - vel_gt[:2].squeeze())

        err_list += [vel_est[0] - vel_gt[0].squeeze()]
        err_y_list += [vel_est[1] - vel_gt[1]]
        omega_gt_list += [vel_gt[5]]
        # Just take nearest IMU yaw measurement for measured yaw
        # Manually checked that the max dt between radar and IMU is around 0.002 seconds,
        # so we can assume yaw stays constant until we get to the radar frame
        imu_idx = np.argmin(np.abs(imu_time - rad_frame.timestamp))
        omega_list += [imu_yaw[imu_idx]]
        vel_list += [vel_gt[0]]
        vel_y_list += [vel_gt[1]]
        est_vel_list += [vel_est[0]]
        est_vel_y_list += [vel_est[1]]
        est_vel_og_list += [vel_est_og[0]]
        est_vel_y_og_list += [vel_est_og[1]]
        timestamps_list += [int(rad_frame.timestamp_micro)]

        # Reshape vel_est to be 2D for saving
        vel_est = vel_est.reshape(-1, 1)
        vel_gt = vel_gt.reshape(-1, 1)
        vel_est_og = vel_est_og.reshape(-1, 1)

        # Form result_vel list to be saved
        result_vel.append([int(rad_frame.timestamp_micro)] + vel_est[0].tolist() + vel_est[1].tolist() + vel_gt[0].tolist() + vel_gt[1].tolist() + vel_est_og[0].tolist() + vel_est_og[1].tolist())

        # Save velocity estimates and error in velocities to csv
        u_gt = vel_gt[0] * np.cos(az_est) + vel_gt[1] * np.sin(az_est)

        df_data = {'seq': seq[0],
                   'frame_time': rad_frame.timestamp_micro,
                'az_time': timestamps,
                'azimuth': azimuths.squeeze(),
                'v_v_gt_0': vel_gt[0]*np.ones(azimuths.shape[0]),
                'v_v_gt_1': vel_gt[1]*np.ones(azimuths.shape[0]),
                'u_gt': u_gt,
                'u_est': u_est}

        df = pd.DataFrame(df_data)
        if not osp.exists(seq_metadata_path):
            df.to_csv(seq_metadata_path, index=False, na_rep='NULL')
        else:
            df.to_csv(seq_metadata_path, mode='a', header=False, index=False, na_rep='NULL')

    # Compute stats for sequence
    df = pd.read_csv(seq_metadata_path)
    plot_results(df, seq_metadata_dir)

    # Print error stats
    err_list = np.array(err_list).squeeze()
    err_y_list = np.array(err_y_list).squeeze()
    omega_gt_list = np.array(omega_gt_list).squeeze()
    omega_list = np.array(omega_list).squeeze()
    vel_list = np.array(vel_list).squeeze()
    vel_y_list = np.array(vel_y_list).squeeze()
    est_vel_list = np.array(est_vel_list).squeeze()
    est_vel_y_list = np.array(est_vel_y_list).squeeze()
    est_vel_og_list = np.array(est_vel_og_list).squeeze()
    est_vel_y_og_list = np.array(est_vel_y_og_list).squeeze()
    timestamps_list = np.array(timestamps_list).squeeze()

    # Estimate pose from estimated velocity and omega
    T_est = T_init
    pos_est = np.zeros((len(est_vel_list), 3))
    pos_est[0] = T_est[:3, 3].squeeze()
    result_pose = []
    result_pose.append([timestamps_list[0]] + np.linalg.inv(T_init).flatten().tolist()[:12])
    for idx in range(len(est_vel_list)-1):
        dt = float(timestamps_list[idx+1] - timestamps_list[idx]) / 10**6
        if use_gt_yaw:
            omega = omega_gt_list[idx]
        else:
            omega = omega_list[idx]
        vel_vec = np.array([est_vel_list[idx]*dt, est_vel_y_list[idx]*dt, 0, 0, 0, omega*dt]).reshape(6, 1)
        T_est = T_est @ se3op.vec2tran(vel_vec)
        # Invert bc boreas processing script expects T_ba
        T_a_s = np.linalg.inv(T_est).flatten().tolist()[:12]
        pos_est[idx+1] = T_est[:3, 3].squeeze()
        # Save new pose
        result_pose.append([timestamps_list[idx+1]] + T_a_s)

    # Save pose results to file
    with open(osp.join(output_dir, 'odometry_result', seq[0] + ".txt"), "+w") as file:
        writer = csv.writer(file, delimiter=' ')
        writer.writerows(result_pose)
        print("Written to file:", osp.join(output_dir, 'odometry_result', seq[0] + ".txt"))

    # Save velocity results to file
    with open(osp.join(output_dir, 'odometry_vel_result', seq[0] + ".txt"), "+w") as file:
        writer = csv.writer(file, delimiter=' ')
        writer.writerows(result_vel)
        print("Written to file:", osp.join(output_dir, 'odometry_vel_result', seq[0] + ".txt"))

    eval_stats(output_dir)

def plot_results(df, result_path):
    u_est = np.asarray(df.u_est)
    u_gt = np.asarray(df.u_gt)
    az_err = (u_est - u_gt)
    az_err[abs(az_err) > 20] = np.nan

    RMSE_total = np.sqrt(np.sum(az_err[~np.isnan(az_err)]**2) / az_err.shape[0])
    percent_nan = np.sum(np.isnan(az_err)) / len(az_err) * 100
    print("Percent nan: {}%".format(round(percent_nan,2)))
    print("RMSE velocity error: {} m/s".format(round(RMSE_total,2)))
    
    # Plot historgram
    # Find Gaussian fit first
    mu, std = scipy.stats.norm.fit(az_err[~np.isnan(az_err)])
    fig = plt.figure()
    plt.hist(az_err[~np.isnan(az_err)], bins=100, density=True, stacked=True)
    plt.title("Histogram of velocity errors")
    plt.xlabel("Point-wise velocity error (m/s)")
    plt.ylabel("Density of Points")
    # Plot Gaussian now
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = scipy.stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'r', linewidth=2)
    gaussian_legend = 'Gaussian fit: mu = {:.2f},  std = {:.2f}'.format(mu, std)
    plt.legend([gaussian_legend])
    plt.savefig(osp.join(result_path, 'err_hist.png'))
    plt.close()

    # Plot mean error as a function of azimuth
    # Compute average error for each azimuth
    azimuths = df.azimuth.unique()
    az_err_per_az = np.zeros(len(azimuths))

    for idx, az in enumerate(azimuths):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            az_err_per_az[idx] = np.sqrt(np.nanmean(az_err[df.azimuth == az]**2))

    fig = plt.figure()
    plt.scatter(azimuths, az_err_per_az)
    plt.title("RMSE velocity error vs azimuth")
    plt.xlabel("Azimuth (rad)")
    plt.ylabel("Velocity error (m/s)")
    plt.savefig(osp.join(result_path, 'err_vs_az.png'))
    plt.close()

    # Plot error as a functin of each vehicle velocity
    # Compute average error for each bin of vehicle velocity
    vel_binned = np.digitize(df.v_v_gt_0, np.linspace(0, 20, 40))
    vel_err = np.zeros(len(np.linspace(0, 20, 40)))
    for idx, vel in enumerate(np.linspace(0, 20, 40)):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            vel_err[idx] = np.sqrt(np.nanmean(az_err[vel_binned == idx]**2))
    
    fig = plt.figure()
    plt.scatter(np.linspace(0, 20, 40), vel_err)
    plt.title("RMSE velocity error vs vehicle velocity (y)")
    plt.xlabel("Vehicle velocity (m/s)")
    plt.ylabel("Velocity error (m/s)")
    plt.savefig(osp.join(result_path, 'err_vs_vel.png'))
    plt.close()

    # Plot distribution of vehicle velocities in the data
    fig = plt.figure()
    plt.hist(df.v_v_gt_1, bins=100)
    plt.title("Histogram of vehicle velocities")
    plt.xlabel("Vehicle velocity (m/s)")
    plt.ylabel("Number of points")
    plt.savefig(osp.join(result_path, 'vel_hist.png'))
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--seq', default=None, type=str, help='path to vtr folder (default: os.getcwd())')
    args = parser.parse_args()

    if args.seq is None:
        print("Please provide a sequence to process")
    else:
        main([args.seq])