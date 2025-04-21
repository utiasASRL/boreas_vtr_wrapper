import numpy as np
import os.path as osp
import os
from pyboreas import BoreasDataset
import cv2
from utils.radar_utils import load_radar


class DoppShiftDataset():


    def __init__(self, sequences, params=None):
        self.seqences = sequences
        self.params = params
        self.del_r_gt = np.array([])
        self.signal_samples = np.array([])

        data_dir = params['data_dir']
        folder_name = params['folder_name']

        bd = BoreasDataset(data_dir, split = None, verbose=False)

        for seq in sequences:
            bd_seq = bd.get_seq_from_ID(seq)
            radar_img_dir = osp.join(data_dir, seq, folder_name)
            for frame in bd_seq.radar_frames:
                body_rate = frame.body_rate
                timestamp_micro = frame.timestamp_micro
                radar_img = cv2.imread(osp.join(radar_img_dir, str(timestamp_micro) + '.png'), cv2.IMREAD_GRAYSCALE)
                fft_data, azimuths, timestamps, up_chrips = load_radar(radar_img)
                print(up_chrips)
                fads

    def __len__(self):
        return self.del_r_gt.shape[0]
    
    def __getitem__(self, index):
        return self.del_r_gt[index], self.signal_samples[index]