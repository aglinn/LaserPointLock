import numpy as np
import cv2
import nfft
import copy
import os
from lmfit import Parameters
from lmfit.models import Gaussian2dModel
import concurrent.futures
from functools import partial
import time
from scipy.stats import skew
import gc
from sys import getsizeof
import multiprocessing

class DataAnalyzer():

    def __init__(self):
        self._cam_1_loaded_com = []
        self._cam_2_loaded_com = []
        self._t1 = []
        self._t2 = []
        self._frequency_domain = None
        self._r0 = [0, 0, 0, 0]
        self._cam_1_com = []
        self._cam_2_com = []
        self._cam_1_skewness = []
        self._cam_2_skewness = []
        self._cam_1_video = None
        self._cam_2_video = None
        self.img1_threshold = 0
        self.img2_threshold = 0
        self.cam_1_gaussian_results = []
        self.cam_1_gaussian_residual = []
        self.cam_2_gaussian_results = []
        self.cam_2_gaussian_residual = []
        self.gaussian_model = None
        self.params = None
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

    def load_data(self, com1, com2, t1, t2):
        """
        Set dx, t1, t2 from loaded data. This allows convenient analysis of pointing data outside of the app.
        """
        self._cam_1_loaded_com = com1
        self._cam_2_loaded_com = com2
        self._t1 = t1
        self._t2 = t2
        return

    def load_acquisition_folder(self, directory: str):
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
                print("neither cam1 nor cam2 are in the file name, I wrote the save to store cam1 or cam2 in the file"
                      " name.")
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

    def load_video_file(self, file_name):
        if 'cam1.avi' in file_name:
            self._cam_1_video = cv2.VideoCapture(file_name)
        elif 'cam2.avi' in file_name:
            self._cam_2_video = cv2.VideoCapture(file_name)
        else:
            print("expected video file name to be cam1.avi or cam2.avi. But it is not?")
        return

    def get_com_from_video(self, cam_num, use_opencv=True):
        if cam_num == 1 and self._cam_1_video is None:
            print("Must load a video file for cam num, ", cam_num)
        if cam_num == 2 and self._cam_2_video is None:
            print("Must load a video file for cam num, ", cam_num)
        if cam_num == 1:
            self._cam_1_com = []
            while True:
                ret, frame = self._cam_1_video.read()
                if ret:
                    # Convert the frame to grayscale
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    self.process_img(cam_num, gray, use_opencv=use_opencv)
                else:
                    break
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        if cam_num == 2:
            self._cam_2_com = []
            while True:
                ret, frame = self._cam_2_video.read()
                if ret:
                    # Convert the frame to grayscale
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    self.process_img(cam_num, gray, use_opencv=use_opencv)
                else:
                    break
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        return

    def watch_video(self, cam_num, as_difference=False, fps=55.0):
        if cam_num == 1 and self._cam_1_video is None:
            print("Must load a video file for cam num, ", cam_num)
        if cam_num == 2 and self._cam_2_video is None:
            print("Must load a video file for cam num, ", cam_num)
        f_0 = None
        window_name = None
        if cam_num == 1:
            i = 0
            while True:
                ret, frame = self._cam_1_video.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    if as_difference and f_0 is None:
                        f_0 = frame
                    if as_difference:
                        window_name = 'cam 1 frame - first frame'
                        diff = np.asarray(frame, dtype='float')-np.asarray(f_0, dtype='float')
                        """diff += 120
                        diff = diff/diff.max()"""
                        cv2.imshow(window_name, diff)
                    else:
                        window_name = 'cam 1 frame'
                        cv2.imshow(window_name, frame)
                    i += 1
                else:
                    break
                if cv2.waitKey(int(1/fps*1000)) & 0xFF == ord('q'):
                    break
        if cam_num == 2:
            i = 0
            while True:
                ret, frame = self._cam_2_video.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    if as_difference and f_0 is None:
                        f_0 = frame
                    if as_difference:
                        window_name = 'cam 2 frame - first frame'
                        diff = np.asarray(frame, dtype='float') - np.asarray(f_0, dtype='float')
                        """diff -= diff.min()
                        diff /= diff.max()"""
                        cv2.imshow(window_name, diff)
                    else:
                        window_name = 'cam 2 frame'
                        cv2.imshow(window_name, frame)
                    i += 1
                else:
                    break
                if cv2.waitKey(int(1/fps*1000)) & 0xFF == ord('q'):
                    break
        cv2.destroyWindow(window_name)
        cv2.waitKey(1)
        if cam_num == 1:
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        elif cam_num == 2:
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        return

    def fit_gaussians(self, cam_num, initial_guess: dict = None, get_residuals=False):
        self.gaussian_model = Gaussian2dModel()
        self.params = self.gaussian_model.make_params()
        if cam_num == 1:
            length = int(self._cam_1_video.get(cv2.CAP_PROP_FRAME_COUNT))
            self.cam_1_gaussian_results = []
        elif cam_num == 2:
            length = int(self._cam_2_video.get(cv2.CAP_PROP_FRAME_COUNT))
            self.cam_2_gaussian_results = []
        tenths = np.arange(0, step=int(length / 10), stop=length)
        if length not in tenths:
            tenths = np.append(tenths, length)
        # Read in all frames for parallel fitting
        frames = []
        while True:
            if cam_num == 1:
                ret, frame = self._cam_1_video.read()
            elif cam_num == 2:
                ret, frame = self._cam_2_video.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frames.append(frame)
            else:
                break
        if len(frames) != length:
            print("Wait a second did not read all frames. Len frames: ", len(frames), " and length: ", length)
        X, Y = np.meshgrid(np.arange(frames[0].shape[1]), np.arange(frames[0].shape[0]))
        # Fit first frame to get initial guesses for all other frames
        if initial_guess is None:
            initial_guess = {'amplitude': 1, 'centerx': frame.shape[0] / 2, 'centery': frame.shape[1] / 2,
                             'sigmax': 10, 'sigmay': 10}
        for key in initial_guess.keys():
            if 'fwhm' in key or 'height' in key:
                continue
            self.params[key].value = initial_guess[key]
        result = self.gaussian_model.fit(frames[0], params=self.params, x=X, y=Y)
        if cam_num == 1:
            self.cam_1_gaussian_results.append(result.params)
            if get_residuals:
                self.cam_1_gaussian_residual.append(np.sum(np.abs(result.residual))/np.sum(np.abs(frames[0])))
            initial_guess = self.cam_1_gaussian_results[0]
        elif cam_num == 2:
            self.cam_2_gaussian_results.append(result.params)
            if get_residuals:
                self.cam_2_gaussian_residual.append(np.sum(np.abs(result.residual))/np.sum(np.abs(frames[0])))
            initial_guess = self.cam_2_gaussian_results[0]
        for key in initial_guess.keys():
            if 'fwhm' in key or 'height' in key:
                continue
            self.params[key].value = initial_guess[key]
        del frames[0]
        print("Starting parallel computing")
        processed_imgs = 1
        t_start = time.monotonic()
        t_0 = time.monotonic()
        f = partial(self.gaussian_model.fit, params=self.params, x=X, y=Y)
        executor = concurrent.futures.ProcessPoolExecutor()
        for result in executor.map(f, frames):
            if cam_num == 1:
                self.cam_1_gaussian_results.append(result.params)
                if get_residuals:
                    self.cam_1_gaussian_residual.append(np.sum(np.abs(result.residual)) /
                                                        np.sum(np.abs(frames[processed_imgs-1])))
            elif cam_num == 2:
                self.cam_2_gaussian_results.append(result.params)
                if get_residuals:
                    self.cam_2_gaussian_residual.append(np.sum(np.abs(result.residual)) /
                                                        np.sum(np.abs(frames[processed_imgs-1])))
            if processed_imgs in tenths:
                percent = np.round(100 * processed_imgs / length, decimals=1)
                print("percent complete: ", percent, "total frames: ", length)
                print("time to complete chunk is ", time.monotonic() - t_start)
                t_start = time.monotonic()
            processed_imgs += 1
            del result
        executor.shutdown()
        # Was the above faster?
        print("total time was ", time.monotonic() - t_0)
        if cam_num == 1 and len(self.cam_1_gaussian_results) != length:
            print("Wait a second did not fit all frames. Len results: ", len(self.cam_1_gaussian_results),
                  " and length: ", length)
        if cam_num == 2 and len(self.cam_2_gaussian_results) != length:
            print("Wait a second did not fit all frames. Len results: ", len(self.cam_2_gaussian_results),
                  " and length: ", length)
        if cam_num == 1:
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        elif cam_num == 2:
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        return

    def get_skewness(self, cam_num):
        """
        Get the average skewness along the two axes of every image in a video file.
        """
        if cam_num == 1:
            length = int(self._cam_1_video.get(cv2.CAP_PROP_FRAME_COUNT))
            self._cam_1_skewness = []
        elif cam_num == 2:
            length = int(self._cam_2_video.get(cv2.CAP_PROP_FRAME_COUNT))
            self._cam_2_skewness = []
        # Read in all frames and calculate skewness
        while True:
            if cam_num == 1:
                ret, frame = self._cam_1_video.read()
            elif cam_num == 2:
                ret, frame = self._cam_2_video.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                skew_x = skew(frame, axis=1)
                skew_y = skew(frame, axis=0)
                if cam_num == 1:
                    self._cam_1_skewness.append(np.asarray([skew_x.mean(), skew_y.mean()]))
                elif cam_num == 2:
                    self._cam_2_skewness.append(np.asarray([skew_x.mean(), skew_y.mean()]))
            else:
                break
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

    def watch_video_minus_gaussian(self, cam_num, initial_guess: dict = None, fps=55.0, show_fit=False,
                                   show_frame=False, norm_diff: str= ''):
        if cam_num == 1 and not self.cam_1_gaussian_results:
            self.fit_gaussians(cam_num, initial_guess=initial_guess)
        elif cam_num == 2 and not self.cam_2_gaussian_results:
            self.fit_gaussians(cam_num, initial_guess=initial_guess)
        ind = -1
        X = None
        while True:
            if cam_num == 1:
                ret, frame = self._cam_1_video.read()
            elif cam_num == 2:
                ret, frame = self._cam_2_video.read()
            if ret:
                ind += 1
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = frame.astype(np.float32)
                if X is None:
                    X, Y = np.meshgrid(np.arange(frame.shape[1]), np.arange(frame.shape[0]))
                if cam_num == 1:
                    fit = self.gaussian_model.eval(self.cam_1_gaussian_results[ind], x=X, y=Y)
                if cam_num == 2:
                    fit = self.gaussian_model.eval(self.cam_2_gaussian_results[ind], x=X, y=Y)
                #fit = np.asarray(fit, dtype=np.uint8)
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
        if cam_num == 1:
            self._cam_1_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
        elif cam_num == 2:
            self._cam_2_video.set(cv2.CAP_PROP_POS_FRAMES, 1 - 1)
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

    def process_img(self, cam_number: int, img: np.ndarray, use_opencv=True):
        """
        Given an image, self.cam_x_img, calculate the center of mass in pixel coordinates
        For now, just find COM, but in the future, do any preprocessing that I want.
        """
        if cam_number == 1:
            cv2.GaussianBlur(img, (0, 0), sigmaX=10, dst=img, borderType=cv2.BORDER_CONSTANT)
            cv2.subtract(img, self.img1_threshold, img)  # Because I am using uint, any negative result is set to 0
            if use_opencv:
                com_x, com_y = self.find_com(img)
            else:
                com_x, com_y = self.find_com_manual(img)
            if com_x is not None:
                com_x += self._r0[1]
                com_y += self._r0[0]
                self.cam_1_com = np.asarray([com_x, com_y])
        elif cam_number == 2:
            cv2.GaussianBlur(img, (0, 0), sigmaX=10, dst=img, borderType=cv2.BORDER_CONSTANT)
            cv2.subtract(img, self.img2_threshold, img)  # Because I am using uint, any negative result is set to 0
            com_x, com_y = self.find_com(img)
            if com_x is not None:
                com_x += self._r0[3]
                com_y += self._r0[2]
                self.cam_2_com = np.asarray([com_x, com_y])
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
