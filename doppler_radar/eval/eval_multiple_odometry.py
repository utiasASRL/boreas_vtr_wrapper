import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os.path as osp
from itertools import accumulate, repeat
import os
import matplotlib

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
matplotlib.rcParams["text.usetex"] = True

from pyboreas.utils.odometry import (
    compute_kitti_metrics,
    get_sequence_poses,
    get_sequence_poses_gt,
    get_sequences,
    get_sequence_velocities,
    get_sequence_velocities_gt,
    compute_vel_metrics,
    get_path_from_Tvi_list
)


def eval_odom(est_dirs, est_labels, gt_dir, seq_name, res_dir, title, ax, c_list):
    # evaluation mode
    dim = 2

    seq = [seq_name + '.txt']

    # get corresponding groundtruth poses
    T_gt, _, seq_lens_gt, crop = get_sequence_poses_gt(gt_dir, seq, dim)

    for idx, est_dir in enumerate(est_dirs):
        # parse sequences
        pred_pose = osp.join(est_dir, "odometry_result")
        seq = get_sequences(pred_pose, ".txt")
        T_pred, times_pred, seq_lens_pred = get_sequence_poses(pred_pose, seq)

        path_pred, path_gt = get_path_from_Tvi_list(T_pred, T_gt)

        # Rotate paths depending on type of sequence to nicely visually align between sequence types
        flip_x = False
        rot_angle = 0
        if title == "Suburbs":
            rot_angle = -0.2
        elif title == "Highway":
            rot_angle = -np.pi/2 - 0.22
        elif title == "Tunnel":
            rot_angle = np.pi/2 - 0.06
            flip_x = True
        elif title == "Skyway":
            rot_angle = np.pi - 0.45
            flip_x = True
        R = np.array([[np.cos(rot_angle), -np.sin(rot_angle)], [np.sin(rot_angle), np.cos(rot_angle)]])
        path_gt[:, :2] = path_gt[:, :2] @ R.T
        path_pred[:, :2] = path_pred[:, :2] @ R.T

        if flip_x:
            path_gt[:, 0] = -path_gt[:, 0]
            path_pred[:, 0] = -path_pred[:, 0]
            

        if idx == 0:
            ax.plot(path_gt[:, 0], path_gt[:, 1], '--', c='#CC3311', linewidth=2.5, label='Groundtruth', rasterized=True)

        ax.plot(path_pred[:, 0], path_pred[:, 1], linewidth=2.5, label=est_labels[est_dirs.index(est_dir)], c=c_list[idx], rasterized=True)

    ax.plot(
            path_gt[0, 0],
            path_gt[0, 1],
            "ks",
            markerfacecolor="none",
            label="Sequence Start",
            rasterized=True
        )
    ax.set_xlabel(r"x [m]", fontsize=20, fontdict={'family': 'serif'})
    # ax.set_ylabel("y [m]")
    ax.axis("equal")
    #ax.legend(loc="upper right")
    ax.set_title(r'\textbf{' + title + '}', fontsize=20, fontdict={'family': 'serif'})
    # Set major and minor grid lines
    ax.grid(which='both')
    # Set major and minor ticks
    ax.minorticks_on()
    # Increase size of tick labels
    ax.tick_params(labelsize=12)


def plot_odom_vel(est_dirs, est_labels, gt_dir, seq_name, res_dir):
    # Set up figure
    fig = plt.figure()
    fig.set_figheight(2)
    fig.set_figwidth(12)

    # evaluation mode
    dim = 2

    seq = [seq_name + '.txt']

    # get corresponding groundtruth poses
    vel_gt, times_gt, seq_lens_gt, crop = get_sequence_velocities_gt(gt_dir, seq, dim)
    vel_gt = np.array(vel_gt).squeeze()
    times_gt = np.array(times_gt)
    times_gt = (times_gt - times_gt[0]) / 1e6

    for idx, est_dir in enumerate(est_dirs):
        # parse sequences
        pred_vel_path = osp.join(est_dir, "odometry_vel_result")
        seq = get_sequences(pred_vel_path, ".txt")
        vel_pred, times_pred, seq_vel_lens_pred = get_sequence_velocities(pred_vel_path, seq, dim)
        vel_pred = np.array(vel_pred).squeeze()
        if idx == 1:
            plt.plot(times_gt, vel_pred[:, 0], linewidth=5.0, label=est_labels[est_dirs.index(est_dir)], c='#2ca02c', rasterized=True)
        else:
            plt.plot(times_gt, vel_pred[:, 0], linewidth=5.0, label=est_labels[est_dirs.index(est_dir)], c ='#1f77b4', rasterized=True)

    plt.plot(times_gt, vel_gt[:, 0], '--', linewidth=3., label='Groundtruth', c='#FF0000', rasterized=True)

    plt.xlabel(r"Time [s]", fontsize=20, fontdict={'family': 'serif'})
    plt.ylabel(r"$v_x$ [m/s]", fontsize=20, fontdict={'family': 'serif'})
    # Set major and minor grid lines
    plt.grid(which='both')
    # Set major and minor ticks
    plt.minorticks_on()
    # Increase size of tick labels
    plt.tick_params(labelsize=12)
    plt.legend(loc="upper right", prop={'size': 14})

    plt.savefig(osp.join(res_dir, "skyway_vel_plot.pdf"), pad_inches=0.1, bbox_inches="tight", transparent=True)


if __name__ == "__main__":
    mpl.rc('font', family='serif')
    
    seq_list = [
        # "boreas-2024-01-09-14-00",      #glen
        "boreas-2024-01-25-11-44",      #glen
        # "boreas-2024-02-13-15-26",      #glen

        "boreas-2024-04-09-12-47",      #hwy7
        # "boreas-2024-01-23-12-32",    #hwy7
        # "boreas-2024-02-13-16-13",    #hwy7
        # "boreas-2024-03-08-12-27",      #hwy7

        #"boreas-2024-02-29-14-39",       #tunnel
        "boreas-2024-02-29-14-53",          #tunnel
        #"boreas-2024-02-29-15-02",         #tunnel
        # "boreas-2024-02-29-15-11",          #tunnel

        "boreas-2024-02-29-11-54",    #skyway
        # "boreas-2024-02-29-12-13",      #skyway
        # "boreas-2024-02-29-12-31",      #skyway
        # "boreas-2024-02-29-12-48",     #skyway
    ]

    c_list = ['#1f77b4', '#ff7f0e', '#2ca02c']

    num_seq = len(seq_list)
    fig, axs = plt.subplots(1, num_seq)# figsize=(6, num_seq * 6))
    fig.set_figheight(6)
    fig.set_figwidth(6*num_seq)

    res_dir = "/home/dl/Documents/phd/dev/doppler_radar/doppler_radar/results/plots"
    if not osp.exists(res_dir):
        os.makedirs(res_dir)
    gt_dir = osp.join("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_data")

    est_labels = [
        "B2 (ICP, Gyro.)",
        "N2 (Dopp., ICP, Gyro.)",
        "N3 (Dopp., Gyro.)"
    ]

    for seq_idx, seq in enumerate(seq_list):
        est_dirs = [
            osp.join("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_results/paper1_results/B2", seq),
            osp.join("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_results/paper1_results/N2", seq),
            osp.join("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_results/paper1_results/N3", seq)
        ]

        if seq == "boreas-2024-01-09-14-00" or seq == "boreas-2024-02-13-15-26" or seq == "boreas-2024-01-25-11-44":
            title = "Suburbs"
        elif seq == "boreas-2024-01-23-12-32" or seq == "boreas-2024-04-09-12-47" or seq == "boreas-2024-02-13-16-13" or seq == "boreas-2024-03-08-12-27":
            title = "Highway"
        elif seq == "boreas-2024-02-29-11-54" or seq == "boreas-2024-02-29-12-13" or seq == "boreas-2024-02-29-12-31" or seq == "boreas-2024-02-29-12-48":
            title = "Skyway"
        elif seq == "boreas-2024-02-29-14-39" or seq == "boreas-2024-02-29-14-53" or seq == "boreas-2024-02-29-15-11":
            title = "Tunnel"

        eval_odom(est_dirs, est_labels, gt_dir, seq, res_dir, title, axs[seq_idx], c_list)

        if seq_idx == 0:
            axs[seq_idx].set_ylabel(r"y [m]", fontsize=20, fontdict={'family': 'serif'})
        if seq_idx == 1:
            axs[seq_idx].legend(loc="lower left", prop={'size': 15})

    plt.savefig(osp.join(res_dir, "multi_plot.pdf"), pad_inches=0.1, bbox_inches="tight")


    # Now plot single velocity output
    seq = "boreas-2024-02-29-11-54"
    est_dirs = [
        osp.join("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_results/paper1_results/B1", seq),
        osp.join("/home/dl/Documents/phd/dev/doppler_radar/data/vtr_results/paper1_results/N1", seq),
    ]
    est_labels = [
        "Baseline (B1)",
        "Ours (N1)"
    ]

    plot_odom_vel(est_dirs, est_labels, gt_dir, seq, res_dir)