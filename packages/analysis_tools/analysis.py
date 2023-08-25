import numpy as np
import cv2
import nfft
import copy
import os
from lmfit.models import Gaussian2dModel
import concurrent.futures
from functools import partial
import time
from scipy.stats import skew
from tqdm import tqdm
from .processing import ProcessingMultithread as Processing
import pickle as pkl
from pandas import read_csv
from scipy.interpolate import interp1d
from scipy.signal import decimate, BadCoefficients
import math
from pathlib import Path
import warnings


def prime_factors(n):
    """Return an array of all prime factors of n"""
    x = []
    # Get all factors of 2
    while n % 2 == 0:
        x.append(2)
        n = n / 2

    # n must be odd at this point
    # so a skip of 2 ( i = i + 2) can be used
    for i in range(3, int(math.sqrt(n)) + 1, 2):
        # while i divides n , add i and divide n
        while n % i == 0:
            x.append(i)
            n = n / i

    # Condition if n is a prime
    # number greater than 2
    if n > 2:
        x.append(n)
    return np.asarray(x, dtype=int)


def get_skew_x_y(frame):
    """
    Get the average skew along x and y coordinates of a frame.
    """
    skew_x = skew(frame, axis=1)
    skew_y = skew(frame, axis=0)
    return np.asarray([skew_x.mean(), skew_y.mean()])


def get_skew_x_y_multithread(frames):
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        for result in executor.map(get_skew_x_y, frames):
            results.append(result)
    return results


def read_img(directory, file_name):
    img = cv2.imread(directory + '/' + file_name, cv2.IMREAD_GRAYSCALE)
    return img


class DataAnalyzer:

    def __init__(self):
        self._cam_1_loaded_com = []
        self._cam_2_loaded_com = []
        self._t1 = []
        self._t2 = []
        self._frequency_domain = None
        self._r0 = [0, 0, 0, 0, 0, 0]
        self._cam_1_com = []
        self._cam_2_com = []
        self._pump_cam_com = []
        self._cam_1_skewness = []
        self._cam_2_skewness = []
        self._pump_cam_skewness = []
        self._cam_1_video = None
        self._cam_2_video = None
        self.pump_video = None
        self.img1_threshold = 0
        self.img2_threshold = 0
        self.pump_img_threshold = 0
        self.cam_1_gaussian_results = []
        self.cam_1_gaussian_residual = []
        self.cam_2_gaussian_results = []
        self.cam_2_gaussian_residual = []
        self.pump_cam_gaussian_results = []
        self.pump_cam_gaussian_residual = []
        self.gaussian_model = Gaussian2dModel()
        self.params = self.gaussian_model.make_params()
        self.temperature_data = None
        self.cam_1_intensity = []
        self.cam_2_intensity = []
        self.pump_cam_intensity = []
        self.power_data = None
        self._temperature_time = None
        self._t_pump = None
        self._t_power = None
        self._cam_1_video_length = None
        self._cam_2_video_length = None
        self._pump_cam_video_length = None
        return

    def reset_data(self):
        """
        Re-init dx, t1, and t2 to empty
        """
        self._cam_1_loaded_com = []
        self._cam_2_loaded_com
        self._t1 = []
        self._t2 = []
        return

    @staticmethod
    def convert_frames_to_video(directory, fps_avg=None):
        fourcc = cv2.VideoWriter_fourcc(*'FFV1')  # Save the file as an mp4
        file_names = os.listdir(directory)
        t_pump = []
        # remove video and time files if they exist
        i = 0
        for file in file_names:
            if 'video' in file:
                del file_names[i]
            elif 'pickle' in file:
                del file_names[i]
            elif 'DS_Store' in file:
                del file_names[i]
            i += 1
        for file in file_names:
            if 'Image' not in file:
                print("found file, ", file)
            if 'Store' in file:
                print("found another file, ", file)
        try:
            sorted_files = sorted(file_names, key=lambda x: int(x.split('_')[1]))
            # Get time coordinate of frames:
            pump_cam_time = []
            for file in sorted_files:
                pump_cam_time.append(file.split('-')[1].split('.')[0])
            pump_cam_time_int = np.asarray(pump_cam_time, dtype=int)
            fps = []
            for t in np.unique(pump_cam_time_int):
                fps.append(np.count_nonzero(pump_cam_time_int == t))
            fps = np.asarray(fps)
            t_pump = []
            i = 0
            fps_avg = int(np.round(fps[1:-1].mean(), decimals=0))
            for t in np.unique(pump_cam_time_int):
                j = 0
                while True:
                    if i == 0:
                        j_offset = fps_avg - fps[i]
                        t_pump.append(i + (j + j_offset) * 1 / fps_avg)
                        j += 1
                    elif i == fps.size - 1:
                        j_offset = 0
                        t_pump.append(i + (j + j_offset) * 1 / fps_avg)
                        j += 1
                    else:
                        t_pump.append(i + j * 1 / fps[i])
                        j += 1
                    if j == fps[i]:
                        i += 1
                        break
            t_pump = np.asarray(t_pump)
            t_pump *= 1000  # use ms units
            fn = directory + '/t_pump.pickle'
            with open(fn, 'wb') as file:
                pkl.dump(t_pump, file)
        except ValueError:
            # Then the pump image files do not have time-stamps appended.
            print("You need to manually assign t_pump!")
            if fps_avg is None:
                print("NOT LOADED: Since pump images do not have time stamps appended, then you will have to specify an"
                      " fps of the video.")
                return
            sorted_files = sorted(file_names, key=lambda x: int(x.split('_')[1].split('.')[0]))
            # Get time coordinate of frames:
        # Create Video File
        img = cv2.imread(directory + '/' + sorted_files[0], cv2.IMREAD_GRAYSCALE)
        size = (img.shape[1], img.shape[0])
        video_file = cv2.VideoWriter(directory+'/video.avi', fourcc, fps_avg, size, False)
        video_file.write(img)
        del sorted_files[0]
        executor = concurrent.futures.ThreadPoolExecutor()
        f = partial(read_img, directory)
        with tqdm(total=len(file_names), desc="Processing Frames", unit="frame") as pbar:
            for img in executor.map(f, sorted_files):
                video_file.write(img)
                pbar.update(1)
        video_file.release()
        executor.shutdown()
        return t_pump

    def load_pump_video(self, directory, fps_avg=None):
        files = os.listdir(directory)
        if 'video.avi' not in files:
            self.t_pump = self.convert_frames_to_video(directory, fps_avg=fps_avg)
        else:
            try:
                fn = directory + '/t_pump.pickle'
                with open(fn, 'rb') as handle:
                    self.t_pump = pkl.load(handle)
            except FileNotFoundError:
                print("You need to manually assign t_pump.")
        file = directory + '/video.avi'
        self.pump_video = cv2.VideoCapture(file)
        return

    def load_data(self, com1, com2, t1, t2):
        """
        Set dx, t1, t2 from loaded data. This allows convenient analysis_tools of pointing data outside of the app.
        """
        self._cam_1_loaded_com = com1
        self._cam_2_loaded_com = com2
        self._t1 = t1
        self._t2 = t2
        return

    def load_acquisition_folder(self, directory: str, num_temp_channels=None):
        if 'acquisition' not in directory:
            print("Expected an acquisition folder which I designed to save with acquisition in the name...?")
            return
        file_names = os.listdir(directory)
        for i in range(len(file_names)):
            file_names[i] = os.getcwd() + '/' + directory + '/' + file_names[i]
        self.load_pointing_file(file_names)
        for file_name in file_names:
            if '.avi' in file_name:
                self.load_video_file(file_name)
            elif 'temperature' in file_name:
                self.load_temperature_data(file_name, num_channels=num_temp_channels)
        return

    def load_temperature_data(self, file_name, num_channels=4):
        data = np.memmap(file_name, dtype='float64', mode='r', order='C')
        self.temperature_data = data.reshape(int(data.size / num_channels), num_channels)
        return

    def load_pointing_file(self, file_names: str):
        """
        Give me a list of the file names to load.
        """
        i = 0
        j = 0
        for file_name in file_names:
            if '.avi' in file_name:
                continue
            elif '.txt' in file_name:
                continue
            elif 'temperature' in file_name:
                continue
            if 'cam1' in file_name:
                cam1_pointing_data_memmap = np.memmap(file_name, dtype='float64', mode='r', order='C')
                cam1_pointing_data_memmap = cam1_pointing_data_memmap.reshape(3, int(cam1_pointing_data_memmap.size / 3))
                if i == 0:
                    cam1_pointing_data = copy.deepcopy(cam1_pointing_data_memmap)
                else:
                    cam1_pointing_data = np.concatenate((cam1_pointing_data, copy.deepcopy(cam1_pointing_data_memmap)), axis=1)
                i += 1
            elif 'cam2' in file_name:
                cam2_pointing_data_memmap = np.memmap(file_name, dtype='float64', mode='r', order='C')
                cam2_pointing_data_memmap = cam2_pointing_data_memmap.reshape(3, int(cam2_pointing_data_memmap.size / 3))
                if j == 0:
                    cam2_pointing_data = copy.deepcopy(cam2_pointing_data_memmap)
                else:
                    cam2_pointing_data = np.concatenate((cam2_pointing_data, copy.deepcopy(cam2_pointing_data_memmap)), axis=1)
                j += 1
            else:
                print("Unexpected file, ", file_name, " did nothing with this file.")
        if cam1_pointing_data[2, :].min() != cam2_pointing_data[2, :].min():
            print("interesting the starting time of the data series are not the same: cam1 = ",
                  cam1_pointing_data[2, :].min(),
                  " meanwhile cam2  = ", cam2_pointing_data[2, :].min())
            print("I define t0 to be the minnimum of both times.")
        if cam1_pointing_data[2, :].min() <= cam2_pointing_data[2, :].min():
            cam1_pointing_data[2, :] -= cam1_pointing_data[2, :].min()
            cam2_pointing_data[2, :] -= cam1_pointing_data[2, :].min()
        else:
            cam1_pointing_data[2, :] -= cam2_pointing_data[2, :].min()
            cam2_pointing_data[2, :] -= cam2_pointing_data[2, :].min()

        # Store data.
        self._cam_1_loaded_com = copy.deepcopy(cam1_pointing_data[0:2, :].transpose())
        self._t1 = copy.deepcopy(cam1_pointing_data[2, :])
        self._cam_2_loaded_com = copy.deepcopy(cam2_pointing_data[0:2, :].transpose())
        self._t2 = copy.deepcopy(cam2_pointing_data[2, :])
        return

    def load_power_data(self, file_name: str):
        """
        file_name is path to the file.
        """
        out = read_csv(file_name, header=3)
        self.power_data = np.asarray(out[out.columns[0]])
        return

    @property
    def frequency_domain(self):
        return self._frequency_domain

    @frequency_domain.setter
    def frequency_domain(self, list):
        """
        Nu is in Hz
        list[0] is Nu for cam 1 dx[:,0:2]
        list[1] is Nu for cam 2 dx[:,2,:]
        """
        self._frequency_domain = list
        return

    @property
    def frequency_data(self):
        """
        Find the frequency data on a uniformly spaced grid from a non-uniformly sampled time domain.
        The NyQuist freq >= the (2 * minnimum dt)^-1 in the time-domain signal.
        The spacing in frequency space is defined by the time window, T.
        So, with dt, we can find the number of points we expect if we sample up to +/-1/2dt_min
        With the above, we can find Nu in 1/[unit time], which is ms here; so scale Nu by 1000 to yield Hz.
        Finally to satisfy Parseval's theorem, we need to scale the resulting frequency data by dt.
        Oh, and we scaled the time coordinate by the time window, because nfft wants -1/2 to 1/2 domain. We choose to
        define t=0 at the center of the time window. A shift in time just adds a linear phase in frequency domain.

        Set the frequency domain property and return a list of frequency data.
        """
        dt1 = self.t1[1:] - self.t1[0:-1]  # Time is in ms
        dt1_min = np.amin(dt1)
        dt2 = self.t2[1:] - self.t2[0:-1]
        dt2_min = np.amin(dt2)
        T1 = self.t1[-1] - self.t1[0]
        T2 = self.t2[-1] - self.t2[0]
        time1 = (self.t1 - np.amin(self.t1)) - T1 / 2  # t=0 at the center of the data
        time2 = (self.t2 - np.amin(self.t2)) - T2 / 2  # t=0 at the center of the data
        N1 = int(np.floor(T1 / dt1_min))
        if N1 % 2 == 1:
            N1 = N1 - 1
        N2 = int(np.floor(T2 / dt2_min))
        if N2 % 2 == 1:
            N2 = N2 - 1
        dNu1 = 1 / T1  # Here mHz
        dNu2 = 1 / T2
        Nu1 = np.linspace(-N1 / 2, N1 / 2 - 1, N1) * dNu1 * 1000.0  # Hz
        Nu2 = np.linspace(-N2 / 2, N2 / 2 - 1, N2) * dNu2 * 1000.0  # Hz
        self.frequency_domain = [Nu1, Nu2]
        FT_cam1_dx = nfft.nfft_adjoint(time1 / T1, self.cam1_loaded_com[:, 0], N1) * T1 / len(self.t1)
        FT_cam1_dy = nfft.nfft_adjoint(time1 / T1, self.cam1_loaded_com[:, 1], N1) * T1 / len(self.t1)
        FT_cam2_dx = nfft.nfft_adjoint(time2 / T2, self.cam2_loaded_com[:, 0], N2) * T2 / len(self.t2)
        FT_cam2_dy = nfft.nfft_adjoint(time2 / T2, self.cam2_loaded_com[:, 1], N2) * T2 / len(self.t2)
        return [FT_cam1_dx, FT_cam1_dy, FT_cam2_dx, FT_cam2_dy]

    @property
    def standard_deviation(self):
        """
        Calculate the standard deviation the dx vector.
        """
        return np.nanstd(self.dx, axis=0)

    @staticmethod
    def norm_data(x, x_avg=None, equalize_means=False):
        """
        Normalize a 1D array (x) to vary between 0 and 1 and scale the 1D array x_avg by the same factors as x
        """
        if equalize_means:
            x_avg += x.mean() - x_avg.mean()
        sub = np.asarray(x).min()
        scale = np.asarray(np.asarray(x) - np.asarray(x).min()).max()
        if x_avg is None:
            return (np.asarray(x) - sub) / scale
        else:
            return (np.asarray(x) - sub) / scale, (np.asarray(x_avg) - sub) / scale

    def get_time(self, data_name: str):
        """
        data_name is a string that corresponds to a property that holds data. This function gets the associated
        time coordinate.
        """
        if 'cam_1' in data_name:
            return self.t1
        elif 'cam_2' in data_name:
            return self.t2
        elif 'pump_cam' in data_name:
            if self.t_pump is None:
                print("You need to assign a time to t_pump property to correlate pump cam data")
                return
            return self.t_pump
        elif 'temperature_data' in data_name:
            if self.temperature_time is None:
                print("You need to assign a time to temperature_time property to correlate temperature data")
                return
            return self.temperature_time
        elif 'power_data' in data_name:
            if self.t_power is None:
                print("You need to assign a time to t_power property to correlate power data")
                return
            return self.t_power
        else:
            print("Hmmm your data has no corresponding time coordinate that I am aware of?")
        return

    def time_coordinate_is_same(self, data1_name, data2_name):
        """
        Check to see if the time-series data being correlated comes from the same sensor and thus has the same time
        coordinate, in which case correlation is much easier.
        """
        if 'cam_1' in data1_name and 'cam_1' in data2_name:
            return True
        elif 'cam_2' in data1_name and 'cam_2' in data2_name:
            return True
        elif 'pump_cam' in data1_name and 'pump_cam' in data2_name:
            return True
        elif 'temperature_data' in data1_name and 'temperature_data' in data2_name:
            return True
        elif 'power_data' in data1_name and 'power_data' in data2_name:
            return True
        else:
            return False

    @staticmethod
    def correlate(x1, x2):
        """ get cov_norm between x1 and x2"""
        assert x1.size == x2.size
        m = np.concatenate((x1.reshape(1, x1.size), x2.reshape(1, x2.size)),
                           axis=0)
        return np.corrcoef(m)

    @staticmethod
    def overlap_time(t1, t2, t_offset, frequency_cutoff=None):
        """Find the region of time that overlaps after applying t_offset
        t1 = t2+t_offset"""
        if t_offset <= 0:
            if t2.max()+t_offset < t1.min():
                print("no overlapping region in time for t_offset, ", t_offset)
                return None, None, None, None, None, None
        else:
            if t2.min()+t_offset > t1.max():
                print("no overlapping region in time for t_offset, ", t_offset)
                return None, None, None, None, None, None
        if t2.min()+t_offset >= t1.min():
            t1_start = t2.min()+t_offset
            t2_start = t2.min()
        else:
            t1_start = t1.min()
            t2_start = t1.min()-t_offset
        if t2.max()+t_offset <= t1.max():
            t1_end = t2.max()+t_offset
            t2_end = t2.max()
        else:
            t1_end = t1.max()
            t2_end = t1.max()-t_offset
        if np.abs(t1_end-t1_start - (t2_end-t2_start)) > 1e-3:
            print("intervals are different!", t1_end-t1_start, t2_end-t2_start)
        if frequency_cutoff is not None:
            dt = 1/frequency_cutoff
            dt1 = np.min(t1[1:] - t1[:-1])
            dt2 = np.min(t2[1:] - t2[:-1])
            if dt > dt1 and dt > dt2:
                # the cutoff frequency only matters if it is lower than the highest sampled frequency in both series
                # if so, give me a time spacing that is multiplied by an integer to give me the desired time spacing
                if dt1 <= dt2:
                    Dt = dt1
                else:
                    Dt = dt2
                N_ds = int(np.ceil(dt/Dt))
                Dt = dt/N_ds
                N = (t1_end - t1_start) / Dt
                t1_end -= (N % N_ds)*Dt
                t2_end -= (N % N_ds)*Dt
                N = (t1_end - t1_start) / Dt
                N = int(np.round(N, decimals=0))
                if N % N_ds > 0:
                    print("remainder: ", N % N_ds)
                t1_interp = np.linspace(t1_start, t1_end, num=N, endpoint=False)
                # N1_ds = int(np.round(dt/(t1_interp[1]-t1_interp[0]), decimals=0))
                t2_interp = np.linspace(t2_start, t2_end, num=N, endpoint=False)
                # N2_ds = int(np.round(dt / (t2_interp[1] - t2_interp[0]), decimals=0))
                t1_ds = np.linspace(t1_start, t1_end, num=int(N / N_ds), endpoint=False)
                # print(N / N_ds, int(N / N_ds), t1_ds.size)
                t2_ds = np.linspace(t2_start, t2_end, num=int(N / N_ds), endpoint=False)
                """if not np.all((t1_interp - t2_interp - t_offset) / Dt < 1e-10):
                            print("Unequal interp times!")
                            print(t_offset, np.max(t1_interp - t2_interp - t_offset), Dt)"""
                return t1_interp, t2_interp, t1_ds, t2_ds, N_ds, N_ds
        dt1 = np.min(t1[1:] - t1[:-1])
        dt2 = np.min(t2[1:] - t2[:-1])
        N1 = (t1_end - t1_start) / dt1
        N2 = (t2_end - t2_start) / dt2
        N1 = int(np.round(N1, decimals=0))
        N2 = int(np.round(N2, decimals=0))
        if N1 > N2:
            N1_ds = int(np.ceil(N1 / N2))
            N2_ds = None
            N1 = int(N1_ds * N2)
        elif N2 > N1:
            N1_ds = None
            N2_ds = int(np.ceil(N2 / N1))
            N2 = int(N2_ds * N1)
        t1_interp = np.linspace(t1_start, t1_end, num=N1, endpoint=False)
        t2_interp = np.linspace(t2_start, t2_end, num=N2, endpoint=False)
        if N1_ds is not None:
            t1_ds = np.linspace(t1_start, t1_end, num=int(N1/N1_ds), endpoint=False)
            t2_ds = t2_interp
        elif N2_ds is not None:
            t2_ds = np.linspace(t2_start, t2_end, num=int(N2/N2_ds), endpoint=False)
            t1_ds = t1_interp
        """if not np.all((t1_interp - t2_interp - t_offset) / Dt < 1e-10):
            print("Unequal interp times!")
            print(t_offset, np.max(t1_interp - t2_interp - t_offset), Dt)"""
        return t1_interp, t2_interp, t1_ds, t2_ds, N1_ds, N2_ds

    @staticmethod
    def down_sample(data, N_ds, t_interp, force_average=False):
        """Try to downsample with decimate. When that fails, switch to downsampling via averaging over
        N_ds samples."""
        t_ds = None
        t_shift = None
        if force_average:
            decimate_success = False
            data_ds = np.mean(data.reshape(int(data.size / N_ds), N_ds), axis=1)
            t_ds = np.mean(t_interp.reshape(int(t_interp.size / N_ds), N_ds), axis=1)
            return decimate_success, data_ds, t_ds, t_shift
        if N_ds is not None:
            warnings.filterwarnings("error")
            try:
                if N_ds <= 13:
                    data_ds = decimate(data, N_ds)
                else:
                    N_primes = prime_factors(N_ds)
                    data_ds = decimate(data, N_primes[0])
                    N_primes = np.delete(N_primes, 0)
                    for n in N_primes:
                        data_ds = decimate(data_ds, n)
                decimate_success = True
            except BadCoefficients:
                decimate_success = False
                data_ds = np.mean(data.reshape(int(data.size/N_ds), N_ds), axis=1)
                t_ds = np.mean(t_interp.reshape(int(t_interp.size/N_ds), N_ds), axis=1)
                dt = t_interp[1]-t_interp[0]
                t_shift = dt*(N_ds*(N_ds+1)/2-N_ds)/N_ds
            warnings.resetwarnings()
        else:
            decimate_success = True
            data_ds = data
        return decimate_success, data_ds, t_ds, t_shift

    def get_correlation(self, data1_name: str, data2_name: str, t_offset: float, data1_param_name:str = None,
                        data1_ind: int=None, data2_param_name:str = None,  data2_ind: int=None, frequency_cutoff=None,
                        get_data=False):
        """
        Get the correlation between two attributes of name data1_name and data2_name with the time coordinates of the
        two time-series 1D data offset by t_offset. If frequency_cutoff is not None, then remove higher frequency data
        from the time-series prior to correlating the two time-series data. if get_data is True, return the data that
        is correlated.
        time units have been ms so far, so frequency cutoff should be specified as MHz
        t_offset is in ms and is defined as follows, t_data1 = t_data2+t_offset
        If selecting data from an array that has multiple indicies, e.g. temperature, then provide which index to select
        by setting corresponding data_ind to that integer index.
        If selecting data from Gaussian fitting results, then you need to provide the parameter name in addition"""
        x1 = self.__getattribute__(data1_name)
        if 'gaussian_results' in data1_name:
            x = []
            for param in x1:
                x.append(param[data1_param_name].value)
            x1 = np.asarray(x)
        x1 = np.asarray(x1)
        if len(x1.shape) == 2:
            x1 = x1[:, data1_ind]
        assert len(x1.shape) == 1
        x2 = self.__getattribute__(data2_name)
        if 'gaussian_results' in data2_name:
            x = []
            for param in x2:
                x.append(param[data2_param_name].value)
            x2 = np.asarray(x)
        x2 = np.asarray(x2)
        if len(x2.shape) == 2:
            x2 = x2[:, data2_ind]
        assert len(x2.shape) == 1
        t1 = self.get_time(data1_name)
        if t1 is None:
            return
        if self.time_coordinate_is_same(data1_name, data2_name):
            """Then no need to get the second time coordinates. Just correlate the two 1D arrays."""
            print("no t_offset applied, simple correlation")
            cov_norm = self.correlate(x1, x2)
            if get_data:
                return cov_norm[1, 0], x1, x2, t1, t1
            else:
                return cov_norm[1, 0]
        t2 = self.get_time(data2_name)
        if t2 is None:
            return
        t1_interp, t2_interp, t1_ds, t2_ds, N1_ds, N2_ds = self.overlap_time(t1, t2, t_offset, frequency_cutoff)
        if t1_interp is None:
            return
        interpolator1 = interp1d(t1, x1, kind='cubic')
        x1_interp = interpolator1(t1_interp)
        x1_dec_success, x1_ds, t_ds, t1_shift = self.down_sample(x1_interp, N1_ds, t1_interp)
        if not x1_dec_success:
            t1_ds = copy.deepcopy(t_ds)
        # print(x1_interp.size, x1_ds.size, x1_interp.size/N1_ds)
        interpolator2 = interp1d(t2, x2, kind='cubic')
        x2_interp = interpolator2(t2_interp)
        x2_dec_success, x2_ds, t_ds, t2_shift = self.down_sample(x2_interp, N2_ds, t2_interp)
        if not x2_dec_success:
            t2_ds = copy.deepcopy(t_ds)
        """
        When one of the series is downsampled with the averaging method and the other uses 
        decimate method successfully, then the time coordinate is shifted relative to eachother. 
        This shift is corrected in the below if section."""
        if not x1_dec_success == x2_dec_success:
            # Then one of the two successfully used decimate and the other used the average method
            if not x1_dec_success:
                # need to fix t2_ds because average method introduces a shift of time coordinate.
                if N2_ds is None:
                    # 2 was not downsampled at all. Shift time coordinate and reinterpolate onto it.
                    t2_interp += t1_shift
                    t2_ds = t2_interp
                    x2_interp = interpolator2(t2_interp)
                    x2_dec_success, x2_ds, _, _ = self.down_sample(x2_interp, N2_ds, t2_interp)
                    if not x2_dec_success:
                        print("Do not understand why decimation failed, when no downsampling occurred?")
                else:
                    # 2 just needs to reforce downsample using average method.
                    _, x2_ds, t2_ds, _ = self.down_sample(x2_interp, N2_ds, t2_interp, force_average=True)
            elif not x2_dec_success:
                # need to fix t1_ds because average method introduces a shift of time coordinate.
                if N1_ds is None:
                    # 1 was not downsampled at all. Shift time coordinate and reinterpolate onto it.
                    t1_interp += t2_shift
                    t1_ds = t1_interp
                    x1_interp = interpolator1(t1_interp)
                    x1_dec_success, x1_ds, _, _ = self.down_sample(x1_interp, N1_ds, t1_interp)
                    if not x1_dec_success:
                        print("Do not understand why decimation failed, when no downsampling occurred?")
                else:
                    # 1 just needs to reforce downsample using average method.
                    _, x1_ds, t1_ds, _ = self.down_sample(x1_interp, N1_ds, t1_interp, force_average=True)
        # confirm that the time coordinates are in fact the same!
        if not np.all((t1_ds - t2_ds - t_offset) / (t1_ds[1]-t1_ds[0]) < 1e-12):
            print("Unequal ds times!")
            print(t_offset, np.max(t1_ds - t2_ds - t_offset), (t1_ds[1]-t1_ds[0]))
        # Now, x1_ds and x2_ds are defined to have the same time coordinates, so I can correlate them
        cov_norm = self.correlate(x1_ds, x2_ds)
        if get_data:
            # print(x1_ds.shape, x2_ds.shape, t1_ds.shape, t2_ds.shape, x1_interp.shape, x2_interp.shape, t1_interp.shape, t2_interp.shape)
            return cov_norm[1, 0], x1_ds, x2_ds, t1_ds, t2_ds, x1_interp, x2_interp, t1_interp, t2_interp
        else:
            return cov_norm[1, 0]

    def sweep_t_offset(self, data1_name: str, data2_name: str, t_offset: np.ndarray, data1_param_name:str = None,
                        data1_ind: int=None, data2_param_name:str = None,  data2_ind: int=None, frequency_cutoff=None):
        executor = concurrent.futures.ThreadPoolExecutor()
        f = partial(self.get_correlation, data1_name, data2_name, data1_param_name=data1_param_name,
                    data1_ind=data1_ind, data2_param_name=data2_param_name, data2_ind=data2_ind,
                    frequency_cutoff=frequency_cutoff)
        corr = []
        with tqdm(total=t_offset.size, desc="Processing correlation at t_offsets", unit="offsets") as pbar:
            for correlation in executor.map(f, t_offset):
                corr.append(correlation)
                pbar.update(1)
        return np.asarray(corr)

    def load_video_file(self, file_name):
        if 'cam1.avi' in file_name:
            self._cam_1_video = cv2.VideoCapture(file_name)
        elif 'cam2.avi' in file_name:
            self._cam_2_video = cv2.VideoCapture(file_name)
        else:
            print("expected video file name to be cam1.avi or cam2.avi. But it is not?")
        return

    def select_cap_device(self, cam_num):
        if cam_num == 1:
            if self._cam_1_video is None:
                print("Must load a video file for cam num, ", cam_num)
                return
            cap = self._cam_1_video
        elif cam_num == 2:
            if self._cam_2_video is None:
                print("Must load a video file for cam num, ", cam_num)
                return
            cap = self._cam_2_video
        elif cam_num == 3:
            if self.pump_video is None:
                print("Must load a video file for cam num, ", cam_num)
                return
            cap = self.pump_video
        return cap

    def reset_cap_frame_pos(self, cam_num):
        if cam_num == 1:
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        elif cam_num == 2:
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        elif cam_num == 3:
            self.pump_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        return

    def get_com_from_video(self, cam_num, use_opencv=True, num_frames_in_batch=1024):
        cap = self.select_cap_device(cam_num)
        if cap is None:
            return
        if cam_num == 1:
            self._cam_1_com = []
        elif cam_num == 2:
            self._cam_2_com = []
        elif cam_num == 3:
            self._pump_cam_com = []
        total_frames = self.get_length_of_video(cam_num)
        ret, frames = self.get_frames(cam_num, num_frames_in_batch)
        true_length = len(frames)
        if not ret:
            print("Whoops no frames to read?")
            return
        # Get COM For all frames
        f = partial(self.process_img, cam_num, use_opencv=use_opencv)
        executor = concurrent.futures.ThreadPoolExecutor()
        results = []
        with tqdm(total=total_frames, desc="Processing Frames", unit="frame") as pbar:
            while True:
                for result in executor.map(f, frames):
                    results.append(result)
                    del result
                    pbar.update(1)
                del frames
                ret, frames = self.get_frames(cam_num, num_frames_in_batch)
                if not ret:
                    break
                else:
                    true_length += len(frames)
        # Store the results (COM Stored in process frame function)
        if cam_num == 1:
            self._cam_1_video_length = true_length
        elif cam_num == 2:
            self._cam_2_video_length = true_length
        elif cam_num == 3:
            self._pump_cam_video_length = true_length
        executor.shutdown()
        self.reset_cap_frame_pos(cam_num)
        # Did I get com of every frame.
        if cam_num == 1:
            if total_frames != len(self.cam_1_com):
                print("Whoops did not find the COM of every frame. len skew=", len(self.cam_1_com),
                      "num frames=", total_frames)
        elif cam_num == 2:
            if total_frames != len(self.cam_2_com):
                print("Whoops did not find the COM of every frame. len skew=", len(self.cam_2_com),
                      "num frames=", total_frames)
        elif cam_num == 3:
            if total_frames != len(self.pump_cam_com):
                print("Whoops did not find the COM of every frame. len skew=", len(self.pump_cam_com),
                      "num frames=", total_frames)
        """with tqdm(total=total_frames, desc="Processing Frames", unit="frame") as pbar:
            while True:
                ret, frame = cap.read()
                if ret:
                    # Convert the frame to grayscale
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    self.process_img(cam_num, gray, use_opencv=use_opencv)
                else:
                    break
                pbar.update(1)
        self.reset_cap_frame_pos(cam_num)"""
        return

    def get_window_name(self, cam_num, as_difference):
        window_name = None
        if cam_num == 1:
            if as_difference:
                window_name = 'cam 1 frame - first frame'
            else:
                window_name = 'cam 1 frame'
        elif cam_num == 2:
            if as_difference:
                window_name = 'cam 2 frame - first frame'
            else:
                window_name = 'cam 2 frame'
        elif cam_num == 3:
            if as_difference:
                window_name = 'pump cam frame - first frame'
            else:
                window_name = 'pump cam frame'
        return window_name

    def watch_video(self, cam_num, as_difference=False, fps=55.0):
        cap = self.select_cap_device(cam_num)
        f_0 = None
        window_name = self.get_window_name(cam_num, as_difference)
        if window_name is None:
            print("Not an elligeable cam_num")
            return
        i = 0
        while True:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if as_difference and f_0 is None:
                    f_0 = frame
                if as_difference:
                    diff = np.asarray(frame, dtype='float') - np.asarray(f_0, dtype='float')
                    cv2.imshow(window_name, diff)
                else:
                    cv2.imshow(window_name, frame)
                i += 1
            else:
                break
            if cv2.waitKey(int(1 / fps * 1000)) & 0xFF == ord('q'):
                break
        cv2.destroyWindow(window_name)
        cv2.waitKey(1)
        self.reset_cap_frame_pos(cam_num)
        return

    def get_frames(self, cam_num, num_frames=1024):
        """
        Read next num_frames frames in a capture object into a list of images. return that list of images.
        """
        cap = self.select_cap_device(cam_num)
        frames = []
        while True:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frames.append(frame)
                if len(frames) == num_frames:
                    break
            else:
                break
        if frames:
            return True, frames
        else:
            return False, None

    @staticmethod
    def get_workers_frames(frames):
        num_workers = os.cpu_count()
        frames_per_worker = int(np.floor(len(frames) / num_workers))
        workers_frames = []
        start_frame = 0
        for i in range(num_workers):
            if i + 1 <= len(frames) % num_workers:
                workers_frames.append(frames[start_frame:start_frame + frames_per_worker + 1])
                start_frame += frames_per_worker + 1
            else:
                workers_frames.append(frames[start_frame:start_frame + frames_per_worker])
                start_frame += frames_per_worker
        return workers_frames

    def get_length_of_video(self, cam_num):
        """
        return the number of frames in a capture object for cam_num.
        """
        if cam_num == 1:
            if self._cam_1_video_length is None:
                length = int(self._cam_1_video.get(cv2.CAP_PROP_FRAME_COUNT))
            else:
                length = self._cam_1_video_length
        elif cam_num == 2:
            if self._cam_2_video_length is None:
                length = int(self._cam_2_video.get(cv2.CAP_PROP_FRAME_COUNT))
            else:
                length = self._cam_2_video_length
        elif cam_num == 3:
            if self._pump_cam_video_length is None:
                length = int(self.pump_video.get(cv2.CAP_PROP_FRAME_COUNT))
            else:
                length = self._pump_cam_video_length
        return length

    def prepare_gaussian_results(self, cam_num, get_residuals):
        if cam_num == 1:
            self.cam_1_gaussian_results = []
            if get_residuals:
                self.cam_1_gaussian_residual = []
            gaussian_results = self.cam_1_gaussian_results
            gaussian_residual = self.cam_1_gaussian_residual
        elif cam_num == 2:
            self.cam_2_gaussian_results = []
            if get_residuals:
                self.cam_2_gaussian_residual = []
            gaussian_results = self.cam_2_gaussian_results
            gaussian_residual = self.cam_2_gaussian_residual
        elif cam_num == 3:
            self.pump_cam_gaussian_results = []
            if get_residuals:
                self.pump_cam_gaussian_residual = []
            gaussian_results = self.pump_cam_gaussian_results
            gaussian_residual = self.pump_cam_gaussian_residual
        return gaussian_results, gaussian_residual

    def fit_gaussians(self, cam_num, initial_guess: dict = None, get_residuals=False, num_reports: int = 10,
                      num_frames_in_batch: int = 1000):

        # Clear the variables if they are already fit, presumably user is intentionally refitting and get targets
        gaussian_results, gaussian_residual = self.prepare_gaussian_results(cam_num, get_residuals)
        # Get the total number of frames to process
        length = self.get_length_of_video(cam_num)
        # Get frames numbers at which to print progress report.
        frame_numbers_to_report = self.prepare_to_report(cam_num=cam_num, num_reports=num_reports)
        # Read in batch of frames for parallel fitting
        ret, frames = self.get_frames(cam_num, num_frames_in_batch)
        if not ret:
            print("Whoops no frames to read?")
            return
        # Initialize objects for fitting
        self.gaussian_model = Gaussian2dModel()
        self.params = self.gaussian_model.make_params()
        X, Y = np.meshgrid(np.arange(frames[0].shape[1]), np.arange(frames[0].shape[0]))
        # Fit first frame to get initial guesses for all other frames
        if initial_guess is None:
            initial_guess = {'amplitude': 1, 'centerx': frames[0].shape[0] / 2, 'centery': frames[0].shape[1] / 2,
                             'sigmax': 10, 'sigmay': 10}
        for key in initial_guess.keys():
            if 'fwhm' in key or 'height' in key:
                continue
            self.params[key].value = initial_guess[key]
        result = self.gaussian_model.fit(frames[0], params=self.params, x=X, y=Y)
        gaussian_results.append(result.params)
        if get_residuals:
            gaussian_residual.append(np.sum(np.abs(result.residual)) / np.sum(np.abs(frames[0])))
        initial_guess = gaussian_results[0]
        for key in initial_guess.keys():
            if 'fwhm' in key or 'height' in key:
                continue
            self.params[key].value = initial_guess[key]
        del frames[0]
        processed_imgs = 1
        t_start = time.monotonic()
        t_start_chunk = copy.deepcopy(t_start)
        f = partial(self.gaussian_model.fit, params=self.params, x=X, y=Y)
        # executor = concurrent.futures.ThreadPoolExecutor()
        executor = concurrent.futures.ProcessPoolExecutor()
        index = 0
        with tqdm(total=length, desc="Processing Frames", unit="frame") as pbar:
            while True:
                for result in executor.map(f, frames):
                    gaussian_results.append(result.params)
                    if get_residuals:
                        gaussian_residual.append(np.sum(np.abs(result.residual)) / np.sum(np.abs(frames[index])))
                    index += 1
                    processed_imgs += 1
                    t_start_chunk = self.report_progress(processed_imgs, frame_numbers_to_report, t_start, t_start_chunk)
                    del result
                    pbar.update(1)
                del frames
                ret, frames = self.get_frames(cam_num, num_frames_in_batch)
                index = 0
                if not ret:
                    break
        executor.shutdown()
        self.reset_cap_frame_pos(cam_num)
        # Make sure I fit every frame and reset captures to first frame
        if cam_num == 1 and len(self.cam_1_gaussian_results) != length:
            print("Wait a second did not fit all frames. Len results: ", len(self.cam_1_gaussian_results),
                  " and length: ", length)
        if cam_num == 2 and len(self.cam_2_gaussian_results) != length:
            print("Wait a second did not fit all frames. Len results: ", len(self.cam_2_gaussian_results),
                  " and length: ", length)
        if cam_num == 3 and len(self.pump_cam_gaussian_results) != length:
            print("Wait a second did not fit all frames. Len results: ", len(self.pump_cam_gaussian_results),
                  " and length: ", length)
        return

    def prepare_to_report(self, cam_num, num_reports: int = 10):
        """
        Get parameters needed to provide reporting on processing progress.
        Will get the frame numbers at which to print the reports.
        inputs: cam_num: Which camera stream is being processed?
        num_reports: num of times reports are made throughout the processing.
        return:
        The integer frame numbers at which to print a report. This will represent total_num_frames/num_reports
        """
        length = self.get_length_of_video(cam_num)
        frame_numbers_to_report = np.arange(0, step=int(length / num_reports), stop=length)
        # Always report on last frame being processed.
        if length not in frame_numbers_to_report:
            frame_numbers_to_report = np.append(frame_numbers_to_report, length)
        return frame_numbers_to_report

    def report_progress(self, num_processed_frames, frame_numbers_to_report, t_start, t_start_chunk):
        """
        When processing videos frame by frame, print an update when num_frames_in_stream/num_reports frames have been
        processed.
        Thus, if num_reports=10 there will be an update at every 10% of the processing.
        Updates should have time between reports, and percent complete.
        inputs: num_frames_in_stream: total frames in video file
        num_reports: num of times reports are made throughout the processing.
        """
        if num_processed_frames in frame_numbers_to_report[:-1]:
            percent = np.round(100 * num_processed_frames / frame_numbers_to_report[-1], decimals=1)
            print("percent complete: ", percent, "total frames: ", frame_numbers_to_report[-1])
            print("time to complete chunk is ", time.monotonic() - t_start_chunk)
            t_start_chunk = time.monotonic()
        elif num_processed_frames == frame_numbers_to_report[-1]:
            percent = np.round(100 * num_processed_frames / frame_numbers_to_report[-1], decimals=1)
            print("percent complete: ", percent, "total frames: ", frame_numbers_to_report[-1])
            print("time to complete chunk is ", time.monotonic() - t_start_chunk)
            print("time to complete all processing is ", time.monotonic()-t_start)
        return t_start_chunk

    def clear_skewness(self, cam_num):
        if cam_num == 1:
            self._cam_1_skewness = []
        elif cam_num == 2:
            self._cam_2_skewness = []
        elif cam_num == 3:
            self._pump_cam_skewness = []
        return

    def get_skewness(self, cam_num, num_reports: int = 10, num_frames_in_batch: int = 1000):
        """
        Get the average skewness along the two axes of every image in a video file.
        """
        t_start = time.monotonic()
        t_start_last_chunk = copy.deepcopy(t_start)
        self.clear_skewness(cam_num)
        length = self.get_length_of_video(cam_num)
        frame_numbers_to_report = self.prepare_to_report(cam_num=cam_num, num_reports=num_reports)
        # Read in batch of frames
        ret, frames = self.get_frames(cam_num, num_frames_in_batch)
        if not ret:
            print("Whoops no frames to read?")
            return
        # Get average skewness for all frames
        executor = concurrent.futures.ThreadPoolExecutor()
        results = []
        processed_imgs = 0
        with tqdm(total=length, desc="Processing Frames", unit="frame") as pbar:
            while True:
                for result in executor.map(get_skew_x_y, frames):
                    results.append(result)
                    processed_imgs += 1
                    t_start_last_chunk = self.report_progress(processed_imgs, frame_numbers_to_report, t_start, t_start_last_chunk)
                    del result
                    pbar.update(1)
                del frames
                ret, frames = self.get_frames(cam_num, num_frames_in_batch)
                if not ret:
                    break
        # Store the results
        if cam_num == 1:
            self._cam_1_skewness = results
        elif cam_num == 2:
            self._cam_2_skewness = results
        elif cam_num == 3:
            self._pump_cam_skewness = results
        executor.shutdown()
        self.reset_cap_frame_pos(cam_num)
        # Did I get skewness of every frame.
        if cam_num == 1:
            if length != len(self._cam_1_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self._cam_1_skewness),
                      "num frames=", length)
        elif cam_num == 2:
            if length != len(self._cam_2_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self._cam_2_skewness),
                      "num frames=", length)
        elif cam_num == 3:
            if length != len(self.pump_cam_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self.pump_cam_skewness),
                      "num frames=", length)
        return

    def clear_intensity(self, cam_num):
        if cam_num == 1:
            self.cam_1_intensity = []
        elif cam_num == 2:
            self.cam_2_intensity = []
        elif cam_num == 3:
            self.pump_cam_intensity = []
        return

    def get_intensity(self, cam_num, num_reports: int = 10, num_frames_in_batch: int = 1000):
        t_start = time.monotonic()
        t_start_last_chunk = copy.deepcopy(t_start)
        self.clear_intensity(cam_num)
        length = self.get_length_of_video(cam_num)
        frame_numbers_to_report = self.prepare_to_report(cam_num=cam_num, num_reports=num_reports)
        pbar = tqdm(total=length, desc="Processing Frames", unit="frame")
        # Read in batch of frames
        ret, frames = self.get_frames(cam_num, num_frames_in_batch)
        true_length = len(frames)
        if not ret:
            print("Whoops no frames to read?")
            return
        # Get average skewness for all frames
        executor = concurrent.futures.ThreadPoolExecutor()
        results = []
        processed_imgs = 0
        while True:
            for result in executor.map(np.sum, frames):
                results.append(result)
                processed_imgs += 1
                t_start_last_chunk = self.report_progress(processed_imgs, frame_numbers_to_report, t_start,
                                                          t_start_last_chunk)
                del result
                pbar.update(1)
            del frames
            ret, frames = self.get_frames(cam_num, num_frames_in_batch)
            if not ret:
                break
            else:
                true_length += len(frames)
        del pbar
        # Store the results
        if cam_num == 1:
            self._cam_1_video_length = true_length
            self.cam_1_intensity = results
        elif cam_num == 2:
            self._cam_2_video_length = true_length
            self.cam_2_intensity = results
        elif cam_num == 3:
            self._pump_cam_video_length = true_length
            self.pump_cam_intensity = results
        executor.shutdown()
        self.reset_cap_frame_pos(cam_num)
        # Did I get skewness of every frame.
        if cam_num == 1:
            if length != len(self.cam_1_intensity):
                print("Whoops did not find the skew of every frame. len skew=", len(self.cam_1_intensity),
                      "num frames=", length)
        elif cam_num == 2:
            if length != len(self.cam_2_intensity):
                print("Whoops did not find the skew of every frame. len skew=", len(self.cam_2_intensity),
                      "num frames=", length)
        elif cam_num == 3:
            if length != len(self.pump_cam_intensity):
                print("Whoops did not find the skew of every frame. len skew=", len(self.pump_cam_intensity),
                      "num frames=", length)
        return

    def select_gaussian_results(self, cam_num):
        if cam_num == 1:
            gaussian_results = self.cam_1_gaussian_results
        elif cam_num == 2:
            gaussian_results = self.cam_2_gaussian_results
        elif cam_num == 3:
            gaussian_results = self.pump_cam_gaussian_results
        return gaussian_results

    def watch_video_minus_gaussian(self, cam_num, initial_guess: dict = None, fps=55.0, show_fit=False,
                                   show_frame=False, norm_diff: str = ''):
        if cam_num == 1 and not self.cam_1_gaussian_results:
            self.fit_gaussians(cam_num, initial_guess=initial_guess)
        elif cam_num == 2 and not self.cam_2_gaussian_results:
            self.fit_gaussians(cam_num, initial_guess=initial_guess)
        elif cam_num == 3 and not self.pump_cam_gaussian_results:
            self.fit_gaussians(cam_num, initial_guess=initial_guess)
        cap = self.select_cap_device(cam_num)
        gaussian_results = self.select_gaussian_results(cam_num)
        ind = -1
        X = None
        while True:
            ret, frame = cap.read()
            if ret:
                ind += 1
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = frame.astype(np.float32)
                if X is None:
                    X, Y = np.meshgrid(np.arange(frame.shape[1]), np.arange(frame.shape[0]))
                fit = self.gaussian_model.eval(gaussian_results[ind], x=X, y=Y)
                fit = fit.astype(np.float32)
                diff = (frame - fit)
                if norm_diff == 'per_pixel':
                    diff /= frame
                elif norm_diff == 'max':
                    diff /= frame.max()
                else:
                    pass
                diff -= diff.min()
                if diff.max() > 255:
                    diff /= diff.max()
                elif diff.max() < 1:
                    diff *= 255
                    print("Difference was less than 1")
                diff = diff.astype(np.uint8)
                frame = frame.astype(np.uint8)
                fit = fit.astype(np.uint8)
                if show_frame:
                    window_name = 'Cam ' + str(cam_num) +' frame'
                    cv2.imshow(window_name, frame)
                if show_fit:
                    window_name = 'Cam ' + str(cam_num) + ' fit Gaussian'
                    cv2.imshow(window_name, fit)
                window_name = 'Cam ' + str(cam_num) + ' frame - fit'
                cv2.imshow(window_name, diff)
            else:
                print("Broke on index, ", ind)
                break
            if cv2.waitKey(int(1 / fps * 1000)) & 0xFF == ord('q'):
                break
        self.reset_cap_frame_pos(cam_num)
        return

    def check_com_from_video_is_same_as_loaded(self, cam_num, threshold=1e-6, use_opencv=True):
        if cam_num == 1 and self._cam_1_video is None:
            print("Must load a video file for cam num, ", cam_num)
        if cam_num == 2 and self._cam_2_video is None:
            print("Must load a video file for cam num, ", cam_num)
        if cam_num == 1 and self._cam_1_loaded_com is None:
            print("Load some com data for camera ", cam_num)
        if cam_num == 2 and self._cam_2_loaded_com is None:
            print("Load some com data for camera ", cam_num)
        if cam_num == 1 and not self.cam_1_com.any():
            self.get_com_from_video(cam_num, use_opencv=use_opencv)
        if cam_num == 2 and not self.cam_2_com.any():
            self.get_com_from_video(cam_num, use_opencv=use_opencv)
        found_same = False
        for i in range(11):
            i -= 5
            if cam_num == 1:
                if i < 0:
                    same = self.cam_1_com[-i:] == self.cam_1_loaded_com[0:i]
                elif i == 0:
                    same = self.cam_1_com == self.cam_1_loaded_com
                else:
                    same = self.cam_1_com[0:-i] == self.cam_1_loaded_com[i:]
            elif cam_num == 2:
                if i < 0:
                    same = self.cam_2_com[-i:] == self.cam2_loaded_com[0:i]
                elif i == 0:
                    same = self.cam_2_com == self.cam2_loaded_com
                else:
                    same = self.cam_2_com[0:-i] == self.cam2_loaded_com[i:]
            if same.all():
                found_same = True
                break
        if found_same and i != 0:
            if cam_num == 1:
                if i < 0:
                    self._cam_1_com = self.cam_1_com[-i:]
                    self._cam_1_loaded_com = self.cam_1_loaded_com[0:i]
                    self._t1 = self.t1[0:i]
                elif i > 0:
                    self._cam_1_com = self.cam_1_com[0:-1]
                    self._cam_1_loaded_com = self.cam_1_loaded_com[i:]
                    self._t1 = self.t1[i:]
            elif cam_num == 2:
                if i < 0:
                    self._cam_2_com = self.cam_2_com[-i:]
                    self._cam_2_loaded_com = self.cam2_loaded_com[0:i]
                    self._t2 = self.t2[0:i]
                elif i > 0:
                    self._cam_2_com = self.cam_2_com[0:-1]
                    self._cam_2_loaded_com = self.cam2_loaded_com[i:]
                    self._t2 = self.t2[i:]
            if i < 0:
                print("Found that camera frames were recorded before camera center of masses were logged. shift is ",
                      np.abs(i))
            elif i > 0:
                print("Found that camera frames were recorded after camera center of masses were logged. shift is ",
                      np.abs(i))
            print("Trimmed data so that the camera frames and loaded com indexes are the same.")
        if cam_num == 1:
            same = self.cam_1_com == self.cam_1_loaded_com
            difference = self.cam_1_com - self.cam_1_loaded_com
            thresh = difference <= threshold
        elif cam_num == 2:
            same = self.cam_2_com == self.cam_2_loaded_com
            difference = self.cam_2_com - self.cam_2_loaded_com
            thresh = difference <= threshold
        if same.all():
            print("The com found from the video stream are the same as the loaded com for camera ", cam_num)
        elif thresh.all():
            print("The difference in the com found from the video stream and the loaded com is less than the specified"
                  " threshold, ", threshold, " for camera ", cam_num)
        else:
            print("The com from the video stream and the loaded coordinates are different. Here is the video stream"
                  "-loaded:...")
            print(difference)
        return

    def get_thresh_r0(self, cam_number):
        if cam_number == 1:
            img_thresh = self.img1_threshold
            r0 = self._r0[0:2]
        elif cam_number == 2:
            img_thresh = self.img2_threshold
            r0 = self._r0[2:4]
        elif cam_number == 3:
            img_thresh = self.pump_img_threshold
            r0 = self._r0[4:6]
        else:
            return None, None
        return img_thresh, r0

    def process_img(self, cam_number: int, img: np.ndarray, use_opencv=True):
        """
        Given an image, self.cam_x_img, calculate the center of mass in pixel coordinates
        For now, just find COM, but in the future, do any preprocessing that I want.
        """
        img_thresh, r0 = self.get_thresh_r0(cam_number)
        if img_thresh is None:
            return
        cv2.GaussianBlur(img, (0, 0), sigmaX=10, dst=img, borderType=cv2.BORDER_CONSTANT)
        cv2.subtract(img, img_thresh, img)  # Because I am using uint, any negative result is set to 0
        if use_opencv:
            com_x, com_y = self.find_com(img)
        else:
            com_x, com_y = self.find_com_manual(img)
        if com_x is not None:
            com_x += r0[1]
            com_y += r0[0]
            self.set_com(cam_number, np.asarray([com_x, com_y]))
        return

    @staticmethod
    def find_com(img_thresh):
        """
        Find the COM of an thresholded image as returned by cv2.threshold
        """
        M = cv2.moments(img_thresh)
        if M['m00'] != 0:
            com_x = M["m01"] / M["m00"]
            com_y = M["m10"] / M["m00"]
            return com_x, com_y
        return None, None

    @staticmethod
    def find_com_manual(img_thresh):
        """
        Find the COM in the naive mannor not using opencv. Slower but to check opencv.
        """
        Y, X = np.meshgrid(np.arange(img_thresh.shape[1]), np.arange(img_thresh.shape[0]), indexing='xy')
        mass = np.sum(img_thresh)
        if mass != 0:
            com_x = np.sum(X*img_thresh)/mass
            com_y = np.sum(Y*img_thresh)/mass
            return com_x, com_y
        else:
            return None, None

    @property
    def t1(self):
        """Camera 1 image timestamp units of ms."""
        return np.asarray(self._t1)

    @t1.setter
    def t1(self, value):
        self._t1.append(value)
        return

    @property
    def t2(self):
        """Camera 2 image timestamp units of ms"""
        return np.asarray(self._t2)

    @t2.setter
    def t2(self, value):
        self._t2.append(value)
        return

    def set_com(self, cam_num, com):
        if cam_num == 1:
            self.cam_1_com = com
        elif cam_num == 2:
            self.cam_2_com = com
        elif cam_num == 3:
            self.pump_cam_com = com
        return

    @property
    def cam_1_com(self):
        return np.asarray(self._cam_1_com)

    @cam_1_com.setter
    def cam_1_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 1
        """
        self._cam_1_com.append(vector)
        return

    @property
    def cam_2_com(self):
        return np.asarray(self._cam_2_com)

    @cam_2_com.setter
    def cam_2_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 2
        """
        self._cam_2_com.append(vector)
        return

    @property
    def pump_cam_com(self):
        return np.asarray(self._pump_cam_com)

    @pump_cam_com.setter
    def pump_cam_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on pump cam
        """
        self._pump_cam_com.append(vector)
        return

    @property
    def cam_1_loaded_com(self):
        return np.asarray(self._cam_1_loaded_com)

    @cam_1_loaded_com.setter
    def cam_1_loaded_com(self, vector):
        # [row, column]
        self._cam_1_loaded_com.append(vector)
        return

    @property
    def cam_2_loaded_com(self):
        return np.asarray(self._cam_2_loaded_com)

    @cam_2_loaded_com.setter
    def cam_2_loaded_com(self, vector):
        # [row, column]
        self._cam_2_loaded_com.append(vector)
        return

    @property
    def cam_1_skewness(self):
        return np.asarray(self._cam_1_skewness)

    @property
    def cam_2_skewness(self):
        return np.asarray(self._cam_2_skewness)

    @property
    def pump_cam_skewness(self):
        return np.asarray(self._pump_cam_skewness)

    @property
    def temperature_time(self):
        return self._temperature_time

    @temperature_time.setter
    def temperature_time(self, t: np.ndarray):
        self._temperature_time = t
        return

    def create_temperature_time(self, t_start: float, t_end: float, num_pts: int):
        self._temperature_time = np.linspace(t_start, t_end, num_pts)
        return

    @ property
    def t_pump(self):
        return self._t_pump

    @t_pump.setter
    def t_pump(self, t: np.ndarray):
        self._t_pump = t
        return

    @property
    def t_power(self):
        return self._t_power

    @t_power.setter
    def t_power(self, t: np.ndarray):
        self._t_power = t
        return

    def store_gaussian_results(self, cam_num: int, directory_name=''):
        results = self.select_gaussian_results(cam_num)
        if directory_name:
            if directory_name[-1] != '/' or directory_name[-1] != '\\':
                directory_name += '/'
        file_name = 'cam_' + str(cam_num) + '_gaussian_results'
        file_name = directory_name + file_name
        with open(file_name, 'wb') as file:
            pkl.dump(results, file)
        if cam_num == 1:
            residuals = self.cam_1_gaussian_residual
        elif cam_num == 2:
            residuals = self.cam_2_gaussian_residual
        elif cam_num == 3:
            residuals = self.pump_cam_gaussian_residual
        file_name = 'cam_' + str(cam_num) + '_gaussian_residuals'
        file_name = directory_name + file_name
        with open(file_name, 'wb') as file:
            pkl.dump(residuals, file)
        return

    def load_attribute(self, prop_name: str, directory: str):
        file_name = prop_name + '.pickle'
        file_names = os.listdir(directory)
        if file_name in file_names:
            ret = False
            with open(directory + '/' + file_name, 'rb') as file:
                data = pkl.load(file)
            if 'com' in prop_name or 'skewness' in prop_name:
                self.__setattr__('_' + prop_name, list(data))
                ret = True
            elif 'gaussian' in prop_name or 'intensity' in prop_name:
                self.__setattr__(prop_name, data)
                ret = True
        else:
            ret = False
            return ret, file_name
        return ret, None

    def load_all_attributes(self, directory: str):
        prop_name = 'cam_1_com'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_2_com'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'pump_cam_com'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_1_skewness'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_2_skewness'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'pump_cam_skewness'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_1_gaussian_results'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_2_gaussian_results'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'pump_cam_gaussian_results'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_1_gaussian_residual'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_2_gaussian_residual'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'pump_cam_gaussian_residual'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_1_intensity'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'cam_2_intensity'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")

        prop_name = 'pump_cam_intensity'
        success, file_name = self.load_attribute(prop_name, directory)
        if not success:
            if file_name is not None:
                print("did not load data for ", prop_name, " because file, ", file_name, ", was not found.")
            else:
                print("did not load data for ", prop_name, " but file, ", file_name, ", was found. Strange!")
        return

    def save_attribute(self, prop_name: str, directory: str):
        data = self.__getattribute__(prop_name)
        try:
            if data:
                ret = True
                file_name = prop_name + '.pickle'
                with open(directory + '/' + file_name, 'wb') as file:
                    pkl.dump(data, file)
            else:
                ret = False
        except ValueError:
            if not data.size == 0:
                ret = True
                file_name = prop_name + '.pickle'
                with open(directory + '/' + file_name, 'wb') as file:
                    pkl.dump(data, file)
            else:
                ret = False
        return ret

    def save_all_attributes(self, directory: str):
        Path(directory).mkdir(parents=True, exist_ok=True)

        prop_name = 'cam_1_com'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_2_com'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'pump_cam_com'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_1_skewness'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_2_skewness'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'pump_cam_skewness'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_1_gaussian_results'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_2_gaussian_results'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'pump_cam_gaussian_results'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_1_gaussian_residual'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_2_gaussian_residual'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'pump_cam_gaussian_residual'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_1_intensity'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'cam_2_intensity'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")

        prop_name = 'pump_cam_intensity'
        success = self.save_attribute(prop_name, directory)
        if not success:
            print("no data for property, ", prop_name, ", so nothing saved for this property.")
        return

    def report_progress_multithread(self, num_processed_frames, batch_size, frame_numbers_to_report, t_start,
                                    t_start_chunk):
        """
        Since multithreading means batches of images will be returned all at once, see if that batch crossed the
        number of frames threshold to report.
        When processing videos frame by frame, print an update when num_frames_in_stream/num_reports frames have been
        processed.
        Thus, if num_reports=10 there will be an update at every 10% of the processing.
        Updates should have time between reports, and percent complete.
        inputs: num_frames_in_stream: total frames in video file
        num_reports: num of times reports are made throughout the processing.
        """
        chk1 = num_processed_frames < frame_numbers_to_report
        chk2 = (num_processed_frames + batch_size) >= frame_numbers_to_report
        chk = chk1 & chk2
        if chk[:-1].any():
            percent = np.round(100 * num_processed_frames / frame_numbers_to_report[-1], decimals=1)
            print("percent complete: ", percent, "total frames: ", frame_numbers_to_report[-1])
            print("time to complete chunk is ", time.monotonic() - t_start_chunk)
            t_start_chunk = time.monotonic()
        elif chk[-1]:
            percent = np.round(100 * num_processed_frames / frame_numbers_to_report[-1], decimals=1)
            print("percent complete: ", percent, "total frames: ", frame_numbers_to_report[-1])
            print("time to complete chunk is ", time.monotonic() - t_start_chunk)
            print("time to complete all processing is ", time.monotonic() - t_start)
        return t_start_chunk

    def get_skewness_processor(self, cam_num, processing_kws):
        """
        Get the average skewness along the two axes of every image in a video file.
        """
        t_start = time.monotonic()
        # Make sure variables are empty since user requested calculation and select which camera cap to use
        if cam_num == 1:
            self._cam_1_skewness = []
            cap = self._cam_1_video
        elif cam_num == 2:
            self._cam_2_skewness = []
            cap = self._cam_2_video

        # Get the total number of frames in the video
        total_frames = self.get_length_of_video(cam_num)

        # Instantiate the processing class with the capture object and the method of processing on each frame
        processing_instance = Processing(get_skew_x_y, **processing_kws)
        # Run the frame processing with a progress bar
        with tqdm(total=total_frames, desc="Processing Frames", unit="frame") as pbar:
            processed_frames = processing_instance.run_processing(capture=cap)
            pbar.update(total_frames)

        # Did I get skewness of every frame.
        if cam_num == 1:
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
            self._cam_1_skewness = processed_frames
            if total_frames != len(self._cam_1_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self._cam_1_skewness),
                      "num frames=", total_frames)
        elif cam_num == 2:
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
            self._cam_2_skewness = processed_frames
            if total_frames != len(self._cam_2_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self._cam_2_skewness),
                      "num frames=", total_frames)
        print("The total processing time was ", time.monotonic() - t_start)
        return

    def get_skewness_slow(self, cam_num, num_reports: int = 10):
        """
        Get the average skewness along the two axes of every image in a video file.
        """
        t_start = time.monotonic()
        t_start_last_chunk = copy.deepcopy(t_start)
        if cam_num == 1:
            self._cam_1_skewness = []
            capture = self._cam_1_video
        elif cam_num == 2:
            self._cam_2_skewness = []
            capture = self._cam_2_video
        length = self.get_length_of_video(cam_num)
        frame_numbers_to_report = self.prepare_to_report(cam_num=cam_num, num_reports=num_reports)
        results = []
        processed_imgs = 0
        while True:
            ret, frame = capture.read()
            if ret:
                results.append(get_skew_x_y(frame))
                processed_imgs += 1
                t_start_last_chunk = self.report_progress(processed_imgs, frame_numbers_to_report, t_start,
                                                          t_start_last_chunk)
            else:
                break
        # Store the results
        if cam_num == 1:
            self._cam_1_skewness = results
        elif cam_num == 2:
            self._cam_2_skewness = results
        # Did I get skewness of every frame.
        if cam_num == 1:
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
            if length != len(self._cam_1_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self._cam_1_skewness),
                      "num frames=", length)
        elif cam_num == 2:
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
            if length != len(self._cam_2_skewness):
                print("Whoops did not find the skew of every frame. len skew=", len(self._cam_2_skewness),
                      "num frames=", length)
        return

