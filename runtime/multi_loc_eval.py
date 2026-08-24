import argparse
import os
import numpy as np
import os.path as osp

from pyboreas.eval.localization import eval_local


def eval_loc(pred_dir, gt, sequences, radar=False, plot=False):
    dim = 2 if radar else 3
    ref_sensor = "radar" if radar else "lidar"
    test_sensor = "radar" if radar else "lidar"
    plot_subdir = "{}-{}".format(ref_sensor, test_sensor)

    all_rmse = []
    num_succeed = 0
    num_total = 0

    for seq in sequences:
        pred_loc_path = osp.join(pred_dir, seq, "localization_result")
        # Each teach sequence is its own reference map
        ref_seq = seq
        seq_plot_dir = osp.join(pred_loc_path, plot_subdir) if plot else None
        if seq_plot_dir is not None:
            os.makedirs(seq_plot_dir, exist_ok=True)

        print("=========================================================")
        print("Evaluating teach sequence: {}".format(seq))

        try:
            errs, gt_seqs = eval_local(
                pred_loc_path,
                gt,
                ref_seq,
                ref_sensor=ref_sensor,
                test_sensor=test_sensor,
                dim=dim,
                plot_dir=seq_plot_dir,
            )
        except Exception as e:
            print("\033[91m" + "Sequence {} failed: {}".format(seq, e) + "\033[0m")
            continue

        # errs: (N_passing_query_seqs, 6) — eval_local already filters > 3m failures
        # gt_seqs contains all query sequences including failures
        rmse_cols = errs[:, :6] if errs.ndim > 1 else errs[:6].reshape(1, 6)
        num_total += len(gt_seqs)
        num_succeed += len(rmse_cols)
        all_rmse.extend(rmse_cols)

    if num_succeed == 0:
        print("\033[91m" + "All sequences failed." + "\033[0m")
        return 0

    all_rmse = np.array(list(all_rmse))
    avg_rmse = np.mean(all_rmse, axis=0)

    print("\n=========================================================")
    print(
        "Average RMSE: long.: {:.3f} m  lat.: {:.3f} m  vert.: {:.3f} m  "
        "roll: {:.3f} deg  pitch: {:.3f} deg  yaw: {:.3f} deg".format(
            avg_rmse[1], avg_rmse[0], avg_rmse[2],
            avg_rmse[4], avg_rmse[3], avg_rmse[5],
        )
    )
    print("Number of successfully evaluated localization runs: {} / {}".format(num_succeed, num_total))

    return 1


if __name__ == "__main__":
    # example call: python multi_loc_eval.py --gt ${VTRRDATA} --pred ${VTRRRESULT} --radar --sequences ${SEQUENCES[@]}
    parser = argparse.ArgumentParser()
    parser.add_argument("--pred", type=str, help="path to prediction files")
    parser.add_argument("--gt", type=str, help="path to groundtruth files")
    parser.add_argument(
        "--radar",
        dest="radar",
        action="store_true",
        help="evaluate radar localization in SE(2)",
    )
    parser.add_argument(
        "--plot",
        dest="plot",
        action="store_true",
        help="save per-sequence plots alongside localization_result (default: False)",
    )
    parser.add_argument(
        "--sequences",
        nargs="+",
        help="list of teach sequence directories to evaluate",
    )

    parser.set_defaults(radar=False, plot=False)
    args = parser.parse_args()

    eval_loc(args.pred, args.gt, args.sequences, args.radar, args.plot)
