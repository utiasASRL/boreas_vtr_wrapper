import matplotlib.pyplot as plt
import numpy as np
from utils.radar_utils import point_to_cart_idx
import scipy
import yaml
from sklearn import linear_model

def load_yaml_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def vel_est_err(x, A, b):
    return b - A @ x

class DopplerExtractor:
    def __init__(self, config_path='config/doppler_config.yaml'):
        config = load_yaml_config(config_path)
        self.radar_res = config['radar']['radar_res'] # New radar data resolution
        self.min_range = config['extraction']['signal']['min_range']   #m
        self.max_range = config['extraction']['signal']['max_range']   #m    if 0 then use max range from radar data

        self.max_range_pix = int(self.max_range / self.radar_res)
        self.min_range_pix = int(self.min_range / self.radar_res)
        if self.max_range_pix == 0:
            self.max_range_pix = -1
        
        self.z_q = config['extraction']['filter']['z_q']
        self.sigma_gauss = config['extraction']['filter']['sigma_gauss']
        self.max_ransac_iter = config['extraction']['ransac']['max_iter']
        self.ransac_threshold = config['extraction']['ransac']['threshold']
        self.ransac_prior_threshold = config['extraction']['ransac']['prior_threshold']
        self.opt_cauchy_rho = config['odometry']['optimization']['cauchy_rho']

        # Load in beta
        del_f = float(config['radar']['del_f']) # Hz
        df_dt = del_f * config['radar']['meas_freq'] # Hz / s
        f_t = float(config['radar']['f_t']) # Hz
        beta_corr_fact = config['extraction']['signal']['beta_corr_fact']
        self.beta_up = beta_corr_fact*(f_t + del_f/2) / df_dt # Hz / s
        self.beta_down = -self.beta_up # Hz / s

    def extract_vehicle_velocity(self, fft_data, azimuths, prior_vel=None, return_u=False):
        u, az = self.extract_doppler(fft_data, azimuths)
        u_ransac, az_ransac = self.ransac(u, az, prior_vel=prior_vel)
        v_est = self.estimate_vel_from_radial(u_ransac, az_ransac, v_init=prior_vel)

        if return_u:
            return v_est, u_ransac, az_ransac
        return v_est

    def extract_doppler(self, fft_data, azimuths):
        u = np.zeros(fft_data.shape[0])
        az = np.zeros(fft_data.shape[0])

        # Minus 1 since we need an ii+1
        for ii in range(fft_data.shape[0]-1):
            u[ii] = self.extract_single_doppler(fft_data[ii], fft_data[ii+1], up_chirp=(ii % 2 == 0), copy=True)
            az[ii] = (azimuths[ii] + azimuths[ii+1]) / 2

        return u, az

    def extract_single_doppler(self, az_0, az_1, up_chirp, copy=True, return_everything=False):
        beta = self.beta_up if up_chirp else self.beta_down
        
        # Create deep copy if desired
        if copy:
            az_0 = az_0.copy()
            az_1 = az_1.copy()

        # Crop azimuths
        if az_0.shape[0] < self.max_range_pix: self.max_range_pix = az_0.shape[0]
        if az_0.shape[0] < self.min_range_pix: raise ValueError("Min range is greater than azimuth length")
        az_0 = az_0[self.min_range_pix:self.max_range_pix]
        az_1 = az_1[self.min_range_pix:self.max_range_pix]
        
        # Run filter
        az_0 = self.cen_filter(az_0)
        az_1 = self.cen_filter(az_1)

        # Check if azimuths has useable return
        if np.all(az_0 == 0) or np.all(az_1 == 0):
            u_i = np.nan
        else:
            corr_jj_i = scipy.signal.correlate(az_0, az_1, mode='full', method='fft')
            del_r_jj = -(np.argmax(corr_jj_i) - (len(az_1) - 1))/2
            del_r = del_r_jj * self.radar_res

            u_i = del_r / beta

        if return_everything:
            return u_i, az_0, az_1, del_r_jj
        return u_i
    
    def ransac(self, u, az, prior_vel = None):
        az_use = az[np.isnan(u) == False]
        u_use = u[np.isnan(u) == False]

        b = u_use.copy()
        A = np.hstack((np.cos(az_use)[:, np.newaxis], np.sin(az_use)[:, np.newaxis]))

        best_num_inlier = 0
        for iter in range(self.max_ransac_iter):
            # Randomly sample 2 points
            idx = np.random.choice(A.shape[0], 2, replace=False)
            A_sample = A[idx]
            b_sample = b[idx]
            # Fit line to sample
            model = linear_model.LinearRegression(fit_intercept=False)
            model.fit(A_sample, b_sample)

            # If prior velocity provided, ignore infeasible solutions
            if (prior_vel is not None) and (np.linalg.norm(model.coef_) - np.linalg.norm(prior_vel) > self.ransac_prior_threshold):
                iter = iter - 1
                continue

            # Compute residuals
            residuals = A @ model.coef_ - b

            # Compute inliers
            num_inlier = np.sum(np.abs(residuals) < self.ransac_threshold)
            if num_inlier > best_num_inlier:
                best_num_inlier = num_inlier
                best_inlier_mask = np.abs(residuals) < self.ransac_threshold

        outlier_mask = np.logical_not(best_inlier_mask)
        u_use[outlier_mask] = np.nan
        az_use[outlier_mask] = np.nan

        # Return full dimension with new nans
        u_ransac = u.copy()
        u_ransac[~np.isnan(u)] = u_use
        az_ransac = az.copy()
        az_ransac[~np.isnan(u)] = az_use

        return u_ransac, az_ransac
    
    def estimate_vel_from_radial(self, u, az, v_init = None):
        # Remove NaNs
        az_use = az[np.isnan(u) == False]
        u_use = u[np.isnan(u) == False]

        # Fit line
        A = np.hstack((np.cos(az_use)[:, np.newaxis], np.sin(az_use)[:, np.newaxis]))
        b = u_use
        if (v_init is None):
            v_init = np.zeros(2)

        vel_est = scipy.optimize.least_squares(vel_est_err, v_init, args=(A, b), loss='cauchy', f_scale=self.opt_cauchy_rho)

        return vel_est.x

    def compute_RMSE(self, u, az, v_vehicle_gt):
        u_use = u[~np.isnan(u)]
        az_use = az[~np.isnan(u)]
        u_gt_in = v_vehicle_gt[0] * np.cos(az_use) + v_vehicle_gt[1] * np.sin(az_use)
        az_rmse = 0.0
        for ii, az in enumerate(az_use):
            u_gt = u_gt_in[ii]
            az_rmse += (u_gt - u_use[ii])**2
        az_rmse = np.sqrt(az_rmse / len(u_use))

        return az_rmse

    def cen_filter(self, s, plot=False, time=False):
        if time: tic = time.time()
        #q = s - scipy.ndimage.median_filter(s, size=w_median)
        q = s - np.mean(s)
        if time:
            toc = time.time()
            print("Median filter time: {}".format(toc-tic))
        
        """
        # Thank you stackoverflow: https://stackoverflow.com/questions/56246970/how-to-apply-a-binomial-low-pass-filter-to-data-in-a-numpy-array
        if time:tic = time.time()
        p = np.zeros_like(q)
        p = scipy.signal.convolve(q, b, mode='same')
        if time:
            toc = time.time()
            print("Binomial filter time: {}".format(toc-tic))
        """

        # Create 1D Gaussian Filter
        p = scipy.ndimage.gaussian_filter(q, self.sigma_gauss, truncate=3.0)

        # Assume values of q below 0 are Gaussian noise with mean 0 and std sigma_q
        if time: tic = time.time()
        sigma_q = np.std(q[q < 0])
        q_dist = scipy.stats.norm(0, sigma_q)
        if time:
            toc = time.time()
            print("Gaussian noise time: {}".format(toc-tic))

        # Scale p by q_dist
        if time: tic = time.time()
        y_low_f = p * (1 - q_dist.pdf(p)/ q_dist.pdf(0))
        y_low_f[y_low_f < 0] = 0
        
        y_high_f = y_low_f + (q - p) * (1 - q_dist.pdf(q - p)/ q_dist.pdf(0))
        y_high_f[y_high_f < 0] = 0
        
        y_thres = y_high_f.copy()
        y_thres[y_high_f < self.z_q*sigma_q] = 0

        if time:
            toc = time.time()
            print("Scaling time: {}".format(toc-tic))
        if plot:
            fig = plt.figure()
            axs = fig.subplots(6, 1)
            axs[0].plot(s, label='Original')
            axs[0].set_title("Original")
            axs[1].plot(q, label='Median filtered')
            axs[1].set_title("Median filtered")
            axs[2].plot(p, label='Binomial filtered')
            axs[2].set_title("Binomial filtered")
            axs[3].plot(y_low_f, label='Scaled Low-F')
            axs[3].set_title("Scaled Low-F")
            axs[4].plot(y_high_f, label='Scaled High-F')
            axs[4].set_title("Scaled High-F")
            axs[5].plot(y_thres, label='Thresholded')
            axs[5].set_title("Thresholded")

        return y_thres

def visualize_doppler(cart_img, pc, vmax=10, vmin=-10, start_fig=True, show_colourbar=True):
    # Need to align the cartesian image with pointcloud xy coordinates
    cart_img = np.rot90(cart_img, k=1, axes=(0,1))
    cart_img = np.flip(cart_img, axis=0)

    # Get cartesian indices of pointcloud
    pc_cart_idx = point_to_cart_idx(np.expand_dims(pc, 0)).squeeze(0)
    non_nan = ~np.isnan(pc[:, 3])

    # Plot cartesian with overlaid points
    if start_fig: fig = plt.figure()
    img = plt.imshow(cart_img, cmap='gray')
    plt.scatter(pc_cart_idx[non_nan,0], pc_cart_idx[non_nan,1], s=5, c=pc[non_nan,3], cmap='bwr_r', vmax=vmax, vmin=vmin, rasterized=True)
    plt.scatter(pc_cart_idx[~non_nan,0], pc_cart_idx[~non_nan,1], s=5, c='orange', marker='D', rasterized=True)

    # Plot arrows from each point to center of image of length equal to velocity
    DX = (cart_img.shape[0]/2 - pc_cart_idx[non_nan,0])
    DY = (cart_img.shape[1]/2 - pc_cart_idx[non_nan,1])
    arrow_vectors = np.stack((DX, DY), axis=1)
    arrow_vectors = 0.005*np.expand_dims(pc[non_nan,3], 1) * arrow_vectors / np.linalg.norm(arrow_vectors, axis=1, keepdims=True)
    plt.quiver(pc_cart_idx[non_nan,0], pc_cart_idx[non_nan,1], arrow_vectors[:,0], arrow_vectors[:,1], pc[non_nan,3], angles='xy', cmap='bwr_r', clim=(vmin, vmax), scale=1, edgecolor='white', linewidth = 0.0, width=0.005, headwidth=3, headlength=4, headaxislength=3, rasterized=True)

    if show_colourbar:
        plt.colorbar(fraction=0.046, pad=0.04)

    plt.xlim([0, cart_img.shape[0]])
    plt.ylim([0, cart_img.shape[1]])
    
    # Remove x and y ticks
    plt.xticks([])
    plt.yticks([])
    
    return img
