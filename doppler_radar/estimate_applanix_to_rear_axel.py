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


def construct_transform(x, z, theta):
    T = se3op.vec2tran(-np.array([0,0,0,0,0,theta]).reshape(-1,1))
    T[0,3] = x
    T[2,3] = z
    return T

def estimate_transform(bd, x_init, z_init, theta_init):
    d = np.array([[0, 1, 0, 0, 0, 0]])

    T_x_a_init = construct_transform(x_init, z_init, theta_init)
    Ad_T_x_a_init = se3op.tranAd(T_x_a_init)

    varpi_a = []
    varpi_r = []
    varpi_x_init = []
    varpi_a_curlyhat = []
    for seq in bd.sequences:
        T_applanix_lidar = seq.calib.T_applanix_lidar
        T_radar_lidar = seq.calib.T_radar_lidar
        T_applanix_radar = T_applanix_lidar @ np.linalg.inv(T_radar_lidar)
        Ad_T_a_r = se3op.tranAd(T_applanix_radar)

        for frame in seq.radar_frames:
            v_r = frame.body_rate
            v_a = Ad_T_a_r @ v_r

            # Don't add measurement if theres no real velocity
            if v_r[0] < 0.3:
                continue
            
            varpi_r.append(v_r)
            varpi_a.append(v_a)
            varpi_x_init.append((Ad_T_x_a_init @ v_a))
            varpi_a_curlyhat.append(se3op.curlyhat(v_a))
            frame.unload_data()

    varpi_r = np.array(varpi_r).squeeze(-1).transpose()
    varpi_a = np.array(varpi_a).squeeze(-1).transpose()
    varpi_x_init = np.array(varpi_x_init).squeeze(-1).transpose()

    v_r_x = varpi_r[0]
    v_r_y = varpi_r[1]
    v_a_x = varpi_a[0]
    v_a_y = varpi_a[1]
    v_x_x = varpi_x_init[0]
    v_x_y = varpi_x_init[1]
    v_angle_r = np.arctan2(v_r_y, v_r_x)
    v_angle_a = np.arctan2(v_a_x, v_a_y) # Y fwd here
    v_angle_x_init = np.arctan2(v_x_y, v_x_x)

    print("Mean vel angle: ", np.mean(v_angle_x_init))

    # Fit line between varpi_x_init[5] and v_angle_x_init
    slope, intercept = np.polyfit(varpi_x_init[5], v_angle_x_init, deg=1)
    print("Slope: ", slope, "Intercept: ", intercept)


    fig = plt.figure()
    plt.plot(v_r_x, label="Radar")
    plt.plot(v_a_y, label='Applanix')
    plt.plot(v_x_x, label='Rear Axel')
    plt.legend()

    fig = plt.figure()
    plt.hist(v_angle_r, bins=100, label="Radar")
    plt.hist(v_angle_a, bins=100, label="Applanix")
    plt.hist(v_angle_x_init, bins=100, label="Rear Axel")
    plt.xlabel("Velocity angle")
    plt.ylabel("Frequency")
    plt.title("Histogram of velocity angles")
    plt.legend()


    fig = plt.figure()
    plt.plot(varpi_r[5], v_angle_r, '.', label="Radar")
    plt.plot(varpi_a[5], v_angle_a, '.', label="Applanix")
    plt.plot(varpi_x_init[5], v_angle_x_init, '.', label="Rear Axel")
    plt.xlabel("Yaw rate")
    plt.ylabel("Velocity angle")
    plt.legend()

    return np.array([x_init, z_init, theta_init]), np.eye(3)

if __name__ == "__main__":

    seq_list = [
        ["boreas-2024-01-23-11-45"],    #glen
        ["boreas-2024-02-08-14-16"],    #glen
        ["boreas-2024-02-13-15-50"],    #hwy7
        ["boreas-2024-03-08-12-13"],    #hwy7
        ["boreas-2024-02-29-14-47"],    #tunnel
        ["boreas-2024-02-29-14-58"],    #tunnel
        # New
        ["boreas-2024-12-03-10-24"],    # glen
        ["boreas-2024-12-03-13-13"],    # hwy 7
        ["boreas-2024-12-03-13-34"],    # hwy 7
        ["boreas-2024-12-04-11-45"],    # skyway
        ["boreas-2024-12-04-14-28"],    # tunnel
        ["boreas-2024-12-04-14-34"],    # tunnel
    ]

    # Load data
    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    bd = BoreasDataset(dataset_dir, split = seq_list, verbose=False)

    # Frames are X -> axel, A -> Applanix, R -> radar
    # State to estimate xi is [x_x^{ax}, 0, z_x^{ax}, 0, 0 theta_{xa}]
    # See https://docs.google.com/document/d/1ba1sV3wmoivJEEtaHZVe6qhOeCWX7H_l7NlhGYXyFQI/edit?tab=t.0 for frame details

    # Initial guess from axel (x fwd, y left, z up) to applanix (y fwd, x right, z up) in axel frame
    # No y since should be 0 y velocity at any y at rear axel
    # Rotation is -90 degrees to rotate from y fwd to x fwd
    # X and Z displacement is taken from pospac data. The Z value removes 0.6937/2 m to get to centre of tire
    x_x_a_init = 0.51
    z_x_a_init = 1.45
    theta_x_a_init = np.pi/2-0.03
    
    xi_x_a, Sigma = estimate_transform(bd, x_x_a_init, z_x_a_init, theta_x_a_init)
    T_x_a = construct_transform(xi_x_a[0], xi_x_a[1], xi_x_a[2])
    print("Transfrom from applanix to rear axel (xi_x_a): ", xi_x_a)
    print("Transform from applanix rear axel SE(3): \n", T_x_a)
    #print("Uncertainty: ", Sigma)

    T_a_l = bd.sequences[0].calib.T_applanix_lidar
    T_r_l = bd.sequences[0].calib.T_radar_lidar
    T_a_r = T_a_l @ np.linalg.inv(T_r_l)
    T_x_r = T_x_a @ T_a_r
    T_x_l = T_x_a @ T_a_l

    print("Transform from radar to rear axel SE(3): \n", T_x_r)
    print("Transform from lidar to rear axel SE(3): \n", T_x_l)
    
    #plt.show()
    