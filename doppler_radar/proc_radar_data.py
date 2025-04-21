import numpy as np
import os
import os.path as osp
from utils.radar_utils import load_radar, radar_polar_to_cartesian, mean_intensity_mask
import cv2
import threading

def gen_polar(seq_path):
    radar_resolution = 0.04381 # New radar data resolution

    rad_seq_path = osp.join(seq_path, 'radar')
    for img_idx, img in enumerate(sorted(os.listdir(rad_seq_path))):
        if not img.endswith('.png'):
            continue
        if img_idx % 100 == 0:
            print(str(img_idx) + "/" + str(len(os.listdir(rad_seq_path))))
        img_path = osp.join(rad_seq_path, img)
        cart_path = osp.join(rad_seq_path, 'cart', img)
        mask_path = osp.join(rad_seq_path, 'mask', img)
        upchirp_path = osp.join(rad_seq_path, 'upchirp', img)
        downchirp_path = osp.join(rad_seq_path, 'downchirp', img)
        if not osp.exists(osp.join(rad_seq_path, 'upchirp')):
            os.mkdir(osp.join(rad_seq_path, 'upchirp'))
        if not osp.exists(osp.join(rad_seq_path, 'downchirp')):
            os.mkdir(osp.join(rad_seq_path, 'downchirp'))

        if osp.exists(cart_path) and osp.exists(mask_path) and osp.exists(upchirp_path) and osp.exists(downchirp_path):
            continue

        radar_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        fft_data, azimuths, timestamps = load_radar(radar_img)
        fft_data[:, :42] = 0

        if not osp.exists(cart_path):
            cart_data = radar_polar_to_cartesian(fft_data, azimuths, radar_resolution, interpolate_crossover=True, fix_wobble=True)
            cv2.imwrite(cart_path, (cart_data * 255.0).astype(np.uint8))
        if not osp.exists(mask_path):
            polar_mask = mean_intensity_mask(fft_data)
            cart_mask = radar_polar_to_cartesian(polar_mask, azimuths, radar_resolution, interpolate_crossover=True, fix_wobble=True)
            cv2.imwrite(mask_path, (cart_mask * 255.0).astype(np.uint8))
        if not osp.exists(upchirp_path):
            up_fft = fft_data[0::2]
            up_za = azimuths[0::2]
            up_cart = radar_polar_to_cartesian(up_fft, up_za, radar_resolution, interpolate_crossover=True, fix_wobble=True)
            cv2.imwrite(upchirp_path, (up_cart * 255.0).astype(np.uint8))
        if not osp.exists(downchirp_path):
            down_fft = fft_data[1::2]
            down_za = azimuths[1::2]
            down_cart = radar_polar_to_cartesian(down_fft, down_za, radar_resolution, interpolate_crossover=True, fix_wobble=True)
            cv2.imwrite(downchirp_path, (down_cart * 255.0).astype(np.uint8))

def main():
    data_dir = '../data'
    dataset_dir = osp.join(data_dir, 'vtr_data')

    # seq_to_proc = [ ["boreas-2023-10-12-11-41"],
    #                 ["boreas-2023-10-12-12-06"],
    #                 ["boreas-2023-10-12-12-25"],
    #                 ["boreas-2023-10-12-12-45"],
    #                 ["boreas-2023-10-12-13-04"] ]

    seq_to_proc = [ ["boreas-2024-01-25-11-44"]]
    
    # seq_to_proc = [ ["boreas-2024-02-29-11-54"],
    #                 ["boreas-2024-02-29-12-13"],
    #                 ["boreas-2024-02-29-12-31"],
    #                 ["boreas-2024-02-29-12-48"],
    #                 ["boreas-2024-02-29-14-39"],
    #                 ["boreas-2024-02-29-14-47"],
    #                 ["boreas-2024-02-29-14-53"],
    #                 ["boreas-2024-02-29-14-58"],
    #                 ["boreas-2024-02-29-15-02"],
    #                 ["boreas-2024-02-29-15-06"],
    #                 ["boreas-2024-02-29-15-11"],
    #                 ["boreas-2024-02-29-15-15"]]

    #seq_to_proc = [ ["boreas-2024-11-02-14-14"],
    #                ["boreas-2024-11-02-14-44"],
    #                ["boreas-2024-10-31-15-41"],
    #                ["boreas-2024-10-31-16-08"] ]

    threads = []

    for seq in seq_to_proc:
        print(seq)
        seq_path = osp.join(dataset_dir, seq[0])
        thread = threading.Thread(target=gen_polar, args=(seq_path,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

if __name__ == "__main__":
    main()