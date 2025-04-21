import matplotlib.pyplot as plt
import argparse
import os
import os.path as osp
import numpy as np

from pyboreas.utils.odometry import (
    compute_vel_metrics
)

def eval_stats(output_dir):
    # Read in odometry pose and velocity info
    vel_file_path = osp.join(output_dir, "odometry_vel_result")
    vel_sequences = [f for f in os.listdir(vel_file_path) if f.endswith('.txt')]
    if len(vel_sequences) == 0 or len(vel_sequences) > 1:
        print('Error: expected exactly one sequence in odometry_vel_result folder')
        return
    vel_file = vel_sequences[0]
    vel_data = np.loadtxt(osp.join(vel_file_path, vel_file))

    # Extract data from vel_data: [timestamp, est_vel, est_vel_y, gt_vel, gt_vel_y, est_vel_og, est_vel_y]
    timestamps = vel_data[:, 0]
    est_vel_x = vel_data[:, 1]
    est_vel_y = vel_data[:, 2]
    gt_vel_x = vel_data[:, 3]
    gt_vel_y = vel_data[:, 4]
    est_vel_x_og = vel_data[:, 5]
    est_vel_y_og = vel_data[:, 6]

    vel_0 = np.zeros_like(est_vel_x)

    # Expand dims to feed to compute_vel_metrics
    vel_pred = np.expand_dims(np.stack((est_vel_x, est_vel_y, vel_0, vel_0, vel_0, vel_0), axis=1), axis=2)
    vel_gt = np.expand_dims(np.stack((gt_vel_x, gt_vel_y, vel_0, vel_0, vel_0, vel_0), axis=1), axis=2)
    timestamps = timestamps

    # compute errors
    crop = [(0, vel_pred.shape[0])]
    v_RMSE, v_mean, v_RMSE_out, v_mean_out = compute_vel_metrics(vel_gt, vel_pred, timestamps, vel_sequences, vel_file_path, 2, crop)

    # print out results
    print("Velocity RMSE: ", v_RMSE, " [m/s, m/s, deg/s]")
    print("Velocity mean: ", v_mean, " [m/s, m/s, deg/s]")
    print("Velocity RMSE w/o outliers: ", v_RMSE_out, " [m/s, m/s, deg/s]")
    print("Velocity mean w/o outliers: ", v_mean_out, " [m/s, m/s, deg/s]")

    # Also make specific raw vel plots for debugging
    plot_time = timestamps / 1e6
    plot_time = plot_time - plot_time[0]

    fig, axs = plt.subplots(1, 2, figsize=(12, 3))
    axs[0].plot(plot_time, est_vel_x_og)
    axs[0].set_xlabel("Time")
    axs[0].set_ylabel("Velocity")
    axs[0].set_title("Raw Fwd. Estimated Velocity")

    axs[1].plot(plot_time, est_vel_y_og)
    axs[1].set_xlabel("Time")
    axs[1].set_ylabel("Velocity")
    axs[1].set_title("Raw Side Estimated Velocity")

    plt.savefig(os.path.join(vel_file_path, vel_sequences[0][:-4] + "_raw_vel.pdf"), pad_inches=0, bbox_inches='tight')
    plt.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # Assuming following path structure:
    # <rosbag name>/metadata.yaml
    # <rosbag name>/<rosbag name>_0.db3
    parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

    args = parser.parse_args()

    eval_stats(args.path)