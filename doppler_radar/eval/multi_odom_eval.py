import argparse
import numpy as np
import matplotlib.pyplot as plt
import os.path as osp
import sys

from pyboreas.utils.odometry import (
    compute_kitti_metrics,
    get_sequence_poses,
    get_sequence_poses_gt,
    get_sequences,
    get_sequence_velocities,
    get_sequence_velocities_gt,
    compute_vel_metrics
)


def eval_odom(pred_dir, gt, sequences, radar=False, velocity=False):
    # evaluation mode
    dim = 2 if radar else 3

    # Store average values
    t_err_avg = 0
    r_err_avg = 0
    v_RMSE_avg = np.zeros(3*(dim-1))
    v_mean_avg = np.zeros(3*(dim-1))

    for seq in sequences:
        pred = osp.join(pred_dir, seq)

        # parse sequences
        pred_pose = osp.join(pred, "odometry_result")
        seq = get_sequences(pred_pose, ".txt")

        T_pred, times_pred, seq_lens_pred = get_sequence_poses(pred_pose, seq)

        # get corresponding groundtruth poses
        T_gt, _, seq_lens_gt, crop = get_sequence_poses_gt(gt, seq, dim)
        
        # compute errors
        t_err, r_err, _ = compute_kitti_metrics(
            T_gt, T_pred, seq_lens_gt, seq_lens_pred, seq, pred_pose, dim, crop
        )

        if velocity:
            # parse sequences
            pred_vel_path = osp.join(pred, "odometry_vel_result")
            seq = get_sequences(pred_vel_path, ".txt")

            vel_pred, times_pred, seq_vel_lens_pred = get_sequence_velocities(pred_vel_path, seq, dim)

            # get corresponding groundtruth poses
            vel_gt, _, seq_lens_gt, crop = get_sequence_velocities_gt(gt, seq, dim)

            # compute errors
            v_RMSE, v_mean, v_RMSE_out, v_mean_out = compute_vel_metrics(vel_gt, vel_pred, times_pred, seq, pred_vel_path, dim, crop)

        # print out results
        print("Evaluated sequences: ", seq)
        print("Overall error: ", t_err, " %, ", r_err, " deg/m")

        if velocity:
            v_RMSE_avg += v_RMSE
            v_mean_avg += v_mean

        t_err_avg += t_err
        r_err_avg += r_err

    t_err_avg /= len(sequences)
    r_err_avg /= len(sequences)
    v_RMSE_avg /= len(sequences)
    v_mean_avg /= len(sequences)

    print("Average overall error: ", t_err_avg, " %, ", r_err_avg, " deg/m")
    if velocity:
        if dim == 2:
            print("Average velocity RMSE: ", v_RMSE_avg, " [m/s, m/s, deg/s]")
            print("Average velocity mean: ", v_mean_avg, " [m/s, m/s, deg/s]")
        else:
            print("Average velocity RMSE: ", v_RMSE_avg, " [m/s, m/s, m/s, deg/s, deg/s, deg/s]")
            print("Average velocity mean: ", v_mean_avg, " [m/s, m/s, m/s, deg/s, deg/s, deg/s]")

    return 1


if __name__ == "__main__":
    # example call: python ${ROOTDIR}/doppler_radar/eval/multi_odom_eval.py --gt ${VTRRDATA} --pred ${VTRRRESULT} --radar --velocity ${VTRRVELRESULT} --sequences ${SEQUENCES[@]}
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pred", type=str, help="path to prediction files"
    )
    parser.add_argument(
        "--gt", type=str, help="path to groundtruth files"
    )
    parser.add_argument(
        "--radar",
        dest="radar",
        action="store_true",
        help="evaluate radar odometry in SE(2)",
    )

    parser.add_argument(
        "--velocity",
        dest="velocity",
        action="store_true",
        help="evaluate velocity (default: False)",
    )
    
    parser.add_argument(
        "--sequences",
        nargs="+",
        help="list of sequences to evaluate",
    )

    parser.set_defaults(radar=False)
    args = parser.parse_args()

    eval_odom(args.pred, args.gt, args.sequences, args.radar, args.velocity)
