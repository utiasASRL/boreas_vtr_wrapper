import argparse
import numpy as np
import matplotlib.pyplot as plt
import os.path as osp
import os
from itertools import accumulate, repeat
from pyboreas import BoreasDataset
from datetime import datetime
import struct
from pylgmath import se3op, so3op



def load_uncertainty(seq_root):
    dst = [
        (1583650800, 1604210399),
        (1615705200, 1636264799),
        (1647154800, 1667714399),
        (1678604400, 1699163999),
        (1710054000, 1730613599),
    ]
    with open(osp.join(seq_root, "applanix", "smrmsg.out"), "rb") as f:
        fc = f.read()
    with open(osp.join(seq_root, "applanix", "ros_and_gps_time.csv"), "r") as f:
        lines = f.readlines()
        start_time = float(lines[1].split(",")[1]) - 5.0
        end_time = float(lines[-1].split(",")[1]) + 5.0
    dt = datetime.fromtimestamp(start_time)
    g2 = (
        dt.isoweekday() * 24 * 3600
        + dt.hour * 3600
        + dt.minute * 60
        + dt.second
        + dt.microsecond * 1e-6
    )
    start_week = round(start_time - g2)
    # get timezone offset:
    # Toronto time is GMT-4 or GMT-5 depending on time of year
    time_zone_offset = 5 * 3600
    for period in dst:
        if period[0] < start_time and start_time < period[1]:
            time_zone_offset = 4 * 3600

    start_gps = start_time + time_zone_offset - start_week
    end_gps = end_time + time_zone_offset - start_week

    t = []
    n = []
    e = []
    d = []
    vn = []
    ve = []
    vd = []
    r = []
    p = []
    h = []

    size = 10 * 8  # size of each line in bytes
    for i in range(len(fc) // size):
        data = struct.unpack("d" * 10, fc[i * size : (i + 1) * size])
        if data[0] < start_gps or data[0] > end_gps:
            continue
        t.append(data[0])
        n.append(data[1])
        e.append(data[2])
        d.append(data[3])
        vn.append(data[4])
        ve.append(data[5])
        vd.append(data[6])
        r.append(data[7])
        p.append(data[8])
        h.append(data[9])

    t = np.array(t)
    t -= t[0]

    return t, n, e, d, vn, ve, vd, r, p, h

def estimate_transform(bd):
    # Projection matrix to strip off down velocity
    D = [[1, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0],
         [0, 0, 0, 1, 0, 0],
         [0, 0, 0, 0, 1, 0],
         [0, 0, 0, 0, 0, 1],
        ]

    T_applanixxfwd_radar = np.array([[ 0.99995263, 0.00973375, 0., 0.0],
                                    [-0.00973375, 0.99995263, 0., 0.0],
                                    [0., 0., -1., 0.495],
                                    [0., 0., 0., 1.]])
    T_axel_applanixxfwd = np.array([[1, 0, 0, 0.65],
                                [0, 1, 0., -0.770],
                                [ 0, 0, 1, 1.8],
                                [ 0, 0, 0, 1]])
    
    T_axel_radar = T_axel_applanixxfwd @ T_applanixxfwd_radar
    Ad_T_axel_radar = se3op.tranAd(T_axel_radar)

    varpi_star = []
    for seq in bd.sequences:
        # Load uncertainty for weighted least squares (maybe not needed)
        # Doesn't currently work, loads in too few measurements for some reason
        # Probably fine without though
        # _, _, _, _, vn, ve, _, roll, pitch, head = load_uncertainty(seq.seq_root)

        for frame in seq.radar_frames:
            v_gt = frame.body_rate

            # Don't add measurement if theres no real velocity
            if v_gt[0] < 0.1:
                continue

            varpi_star.append(D @ (Ad_T_axel_radar @ v_gt))
            #varpi_star.append((v_gt))
            frame.unload_data()

    varpi_star = np.array(varpi_star).reshape(-1, 5)

    # Estimate transform
    cauchy_rho = 0.2
    max_iter = 100
    xi_x_r = np.array([0.65, 2.24, 0])
    del_xi_x_r = 100*np.ones(3)
    weight = np.ones(varpi_star.shape[0])
    # for ii in range(max_iter):
    #     # Define shortcuts
    #     r1 = xi_x_r[0]
    #     r3 = xi_x_r[1]
    #     theta = xi_x_r[2]
    #     C1 = np.cos(theta)
    #     C2 = np.sin(theta)

    #     # Form Jacobian
    #     H_xi = np.zeros((5, 3))
    #     H_xi[4, 0] = -1
    #     H_xi[2, 1] = C1
    #     H_xi[3, 1] = -C2
    #     H_xi[0, 2] = C1
    #     H_xi[1, 2] = -C2
    #     H_xi[2, 2] = -r3*C2
    #     H_xi[3, 2] = -r3*C2
    #     H = varpi_star @ H_xi

    #     # Form error
    #     e = -np.array([C2, C1, r3*C1, -r3*C2, -r1]) @ varpi_star.transpose()

    #     # Use cauchy to update weight
    #     weight = 1 / (1 + (e / cauchy_rho)**2)
    #     W_inv = np.diag(1/weight)

    #     # Solve
    #     del_xi_x_r = np.linalg.inv(H.transpose() @ W_inv @ H) @ H.transpose() @ W_inv @ e
    #     xi_x_r += del_xi_x_r

    #     print("Iteration: ", ii, "Error: ", np.linalg.norm(e), "Step: ", np.linalg.norm(del_xi_x_r))

    #     if np.linalg.norm(del_xi_x_r) < 1e-6:
    #         break

    # Only estimate yaw change
    for ii in range(max_iter):
        # Define shortcuts
        # r1 = xi_x_r[0]
        # r3 = xi_x_r[1]
        r1 = 0
        r3 = 0
        theta = xi_x_r[2]
        C1 = np.cos(theta)
        C2 = np.sin(theta)

        # Form Jacobian
        H_xi = np.zeros((5, 1))
        H_xi[0, 0] = C1
        H_xi[1, 0] = -C2
        H_xi[2, 0] = -r3*C2
        H_xi[3, 0] = -r3*C2
        H = varpi_star @ H_xi

        # Form error
        e = np.array([C2, C1, r3*C1, -r3*C2, -r1]) @ varpi_star.transpose()

        # Use cauchy to update weight
        weight = 1 / (1 + (e / cauchy_rho)**2)
        W_inv = np.diag(1/weight)

        # Solve
        del_xi_x_r = np.linalg.inv(H.transpose() @ W_inv @ H) @ H.transpose() @ W_inv @ e
        xi_x_r[2] -= del_xi_x_r

        print("Iteration: ", ii, "Error: ", np.linalg.norm(e), "Step: ", np.linalg.norm(del_xi_x_r))

        if np.linalg.norm(del_xi_x_r) < 1e-6:
            break

    print("Variance of error: ", np.std(e)**2)
    # Plot histogram of errors
    fig = plt.figure()
    plt.hist(e, bins=100)
    plt.xlabel("Error")
    plt.ylabel("Frequency")
    plt.title("Histogram of errors")
    plt.show()

    #xi_x_r[2] = so3op.rot2vec(C_x_r)[2]

    return xi_x_r, np.linalg.inv(H.transpose() @ W_inv @ H)

if __name__ == "__main__":

    seq_list = [
        ["boreas-2024-01-23-11-45"],      #glen
        ["boreas-2024-02-08-14-16"],      #glen
        ["boreas-2024-02-13-15-50"],      #hwy7
        ["boreas-2024-03-08-12-13"],      #hwy7
        ["boreas-2024-02-29-14-47"],      #tunnel
        ["boreas-2024-02-29-14-58"],      #tunnel
    ]

    # Load data
    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    bd = BoreasDataset(dataset_dir, split = seq_list, verbose=False)

    # xi is [r1, r3, theta]
    xi_x_r, Sigma = estimate_transform(bd)
    # T_x_r = np.array([[np.cos(xi_x_r[2]), -np.sin(xi_x_r[2]), 0, xi_x_r[0]],
    #                     [np.sin(xi_x_r[2]), np.cos(xi_x_r[2]), 0, 0],
    #                     [0, 0, 1, xi_x_r[1]],
    #                     [0, 0, 0, 1]])
    T_x_r = se3op.vec2tran(np.array([0, 0, 0, 0, 0, xi_x_r[2]]).reshape(-1,1))

    print("Transfrom from radar to rear axel (xi_x_r): ", xi_x_r)
    print("Uncertainty: ", Sigma)
    print("Transform to rear axel SE(3): ", T_x_r)
    