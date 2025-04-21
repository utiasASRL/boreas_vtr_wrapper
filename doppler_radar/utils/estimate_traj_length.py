
import os.path as osp
from pyboreas import BoreasDataset
import numpy as np
from pylgmath import se3op

if __name__ == "__main__":
    seq_list = [
        # Old data
        # ["boreas-2024-01-09-14-00"],    # glen
        # ["boreas-2024-01-25-11-44"],    # glen
        # ["boreas-2024-02-13-15-26"],    # glen
        # ["boreas-2024-02-21-12-36"],    # glen

        # ["boreas-2024-04-09-12-47"],    # hwy7
        # ["boreas-2024-01-23-12-32"],    # hwy7
        # ["boreas-2024-02-13-16-13"],    # hwy7
        # ["boreas-2024-03-08-12-27"],    # hwy7

        # New data
        ["boreas-2024-12-03-12-54"],    # glen
        ["boreas-2025-01-08-10-59"],    # glen
        ["boreas-2025-01-08-11-22"],    # glen
        ["boreas-2025-01-08-12-28"],    # glen

        ["boreas-2024-12-10-12-07"],    # hwy7
        ["boreas-2024-12-10-12-24"],    # hwy7
        ["boreas-2024-12-10-12-38"],    # hwy7
        ["boreas-2024-12-10-12-56"],    # hwy7

        ["boreas-2024-12-04-14-38"],    # tunnel
        ["boreas-2024-12-04-14-44"],    # tunnel
        ["boreas-2024-12-04-14-50"],    # tunnel
        ["boreas-2024-12-04-14-59"],    # tunnel

        ["boreas-2024-12-04-11-56"],    # skyway
        ["boreas-2024-12-04-12-08"],    # skyway
        ["boreas-2024-12-04-12-19"],    # skyway
        ["boreas-2024-12-04-12-34"],    # skyway
    ]

    data_dir = '../../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')
    bd = BoreasDataset(dataset_dir, split = seq_list, verbose=False)
    
    tot_distance = 0
    for seq in bd.sequences:
        T_prev = None
        seq_dist = 0
        for frame in seq.radar_frames:
            if T_prev is None:
                T_prev = frame.pose
            else:
                delta_T = np.linalg.inv(frame.pose) @ T_prev
                delta_xi = se3op.tran2vec(delta_T).squeeze()
                seq_dist += np.abs(np.linalg.norm(delta_xi[:3]))
                T_prev = frame.pose
        print(seq.ID, " distance: ", np.round(seq_dist/1000,1), "km")
        tot_distance += seq_dist
    
    print("Average distance: ", np.round(tot_distance/1000/len(bd.sequences),1), "km")
    print("Total distance: ", np.round(tot_distance/1000,1), "km")
