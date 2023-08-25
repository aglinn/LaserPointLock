import numpy as np
import random
from abc import ABC, abstractmethod
import time


"""
Old Classes Unused.
"""


class Camera(ABC):
    @abstractmethod
    def update_frame(self):
        pass

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def set_exposure_time(self, time):
        pass

    @abstractmethod
    def set_resolution(self, res):
        pass

    @abstractmethod
    def set_gain(self, gain):
        pass

    @abstractmethod
    def set_decimation(self, gain):
        pass

    @property
    @abstractmethod
    def exposure_time(self):
        pass

    @property
    @abstractmethod
    def gain(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def apply_ROI(self):
        pass

    @property
    @abstractmethod
    def time(self):
        """
        time that the image was acquired at in ms.
        """
        pass

    @abstractmethod
    def setup_camera_for_image_acquisition(self):
        pass


class FakeCamera(Camera):
    """Multithreaded fake camera. Must subclass QRunnable to utilize Qt multithreading."""
    def __init__(self, **kwargs):
        """
        module_no
        serial_no
        date
        res
        exposure_times
        gain
        decimation
        """
        super().__init__()
        self.module_no = 'SCE-B013-U'
        self.serial_no = kwargs.get('serial_no',
                                    '{0:02}-{1:06}'.format(random.randint(10, 99), random.randint(0, 999999)))
        self.date = '011219'

        self.res = kwargs.get('res', (1024, 1280))
        self._exposure_time = kwargs.get('exposure_times', 0.05)
        self._gain = kwargs.get('gain', 1)
        self.decimation = kwargs.get('decimation', 1)
        self.xcen = kwargs.get('xcen', 0)
        self.ycen = kwargs.get('ycen', 0)
        self.comm_time = 0.2  # minimum time required to get one frame
        self.xres = self.res[0]
        self.yres = self.res[1]
        self._grid = np.empty(0)
        self.set_resolution(self.res)
        self.set_exposure_time(self.exposure_time)
        self.frame = np.zeros(self.res)

        # self.run_thread = CameraThread(self)
        # self.signals = CameraSignals()

    def set_resolution(self, res):
        self.xres = res[0]
        self.yres = res[1]
        self._grid = np.meshgrid((np.arange(self.xres) - self.xres / 2) // self.decimation,
                                 (np.arange(self.yres) - self.yres / 2) // self.decimation, indexing='ij')

    def set_exposure_time(self, exp_time):
        self.exposure_time = exp_time

    def set_gain(self, gain):
        self.gain = gain * 8

    def set_decimation(self, value):
        if isinstance(value, bool):
            self.decimation = 2 if value else 1
            self.set_resolution(self.res)
        else:
            raise ValueError("Decimation must be a boolean")

    def get_frame(self):
        return self.frame

    def pixel_to_um(self, pixels):
        return pixels * 5.5 * self.decimation

    def update_frame(self):
        """This code simulates a laser spot. Change this code to communicate with camera and get camera image.
        As a proof of concept, there is a simulated sleep in this code, which would block execution when not
        multithreaded."""
        # Generate a new center
        self.xcen = min(self.xres // 2, max(-self.xres // 2, self.xcen + random.randint(-10, 10)))
        self.ycen = min(self.yres // 2, max(-self.yres // 2, self.ycen + random.randint(-10, 10)))
        # Simulate a laser spot

        width = 0.02 * self._grid[0].shape[0]
        image = np.round(230 * np.exp(
            -((self._grid[0] - self.xcen) ** 2 + (self._grid[1] - self.ycen) ** 2) / (2 * width ** 2))).astype(np.uint8)
        image += np.round(10*np.random.random(image.shape)).astype(np.uint8)
        time.sleep(self.comm_time)  # simulate communication time
        self.frame = image

    def apply_ROI(self):
        pass

    def time(self):
        return 0
