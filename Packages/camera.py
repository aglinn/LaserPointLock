import numpy as np
import re
import random
from typing import List, Dict, Set, Tuple
from abc import ABC, abstractmethod

from PIL import Image

from Packages.MightexCameraAPI import MightexEngine, MightexCamera
from Packages.Errors import DeviceNotFoundError

class CameraManager():

    def __init__(self):
        # Create resource manager to talk to devices

        self.MightexAPI = None
        

        self.serialList = []
        self.cameraList = []

    def EnableMightexAPI(self):
        self.MightexAPI = MightexEngine()
    
    def getCameraList(self):
        return self.cameraList

    def getCamera(self, index):
        return self.cameraList[index]

    # Returns None if port is already in use
    def addCamera(self, serial_no: str, kind: str):
        # ALl cameras in the list must be unique
        if self.serialList.index(serial_no):
            return None
        
        c: Camera = None
        if kind == "Mightex":
            # Try to add camera from serial_no parameter
            for i, ser_no in self.MightexAPI.serial_no:
                if ser_no == serial_no:
                    self.serialList.append(serial_no)
                    c = MightexCamera(self.MightexAPI, serial_no)
                    
            if not c:
                raise DeviceNotFoundError("Invalid serial number!")
            return None
                    
        else:
            c = FakeCamera()

        self.cameraList.append(c)
        return c

    def removeCamera(self, index):
        for i, c in self.cameraList:
            if c.kind == "Mightex":
                c.deactivate_cam()
            self.serialList.remove(i)
            self.cameraList.remove(i)

    def removeAllCameras(self):
        for i in range(len(self.cameraList)):
            self.removeCamera(i)


# Abstract Camera
class Camera(ABC):

    kind = "Camera"

    def __init__(self):
        self.frame = None
        self.serial_no = ""

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


class FakeCamera(Camera):
    kind = "Camera"

    def __init__(self):
        self.frame = None
        self.serial_no = ""


    # Generate random solid color for fake frame
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



from PyQt5 import QtCore
import time
from flirpy.camera.boson import Boson

lock = QtCore.QReadWriteLock()




"""
# TODO: multithread the frame grabbing
class MightexCamera(Camera):
    def __init__(self, engine: MightexEngine, serial_no: str):
        self.engine: MightexEngine = engine
        if serial_no not in self.engine.serial_no:
            raise DeviceNotFoundError("Serial number {} is not connected to computer".format(serial_no))
        self.serial_no = serial_no
        self.frame = None
        # self.run_thread = CameraThread(self)
        # self.signals = CameraSignals()

    def get_frame(self):
        return self.frame

    def update_frame(self):
        #lock.lockForRead()
        # print("serial number just before calling the engine get frame function:",self.serial_no)
        self.frame = self.engine.get_frame(self.serial_no, 1)
        #lock.unlock()

    def set_exposure_time(self, time):
        pass

    def set_resolution(self, res):
        # self.engine._setResolution(self.engine.dev_num[self.serial_no], res[0], res[1], 0)
        pass

    def set_gain(self, gain):
        pass

    def set_decimation(self, gain):
        pass

    @property
    def exposure_time(self):
        return 0.05

    @property
    def gain(self):
        return 1

    @property
    def dev_num(self):
        return self.engine.dev_num[self.serial_no]
"""


class FakeCamera(Camera):
    """Multithreaded fake camera. Must subclass QRunnable to utilize Qt multithreading."""
    def __init__(self, **kwargs):
        """
        module_no
        serial_no
        date
        res
        exposure_time
        gain
        decimation
        """
        super().__init__()
        self.module_no = 'SCE-B013-U'
        self.serial_no = kwargs.get('serial_no',
                                    '{0:02}-{1:06}'.format(random.randint(10, 99), random.randint(0, 999999)))
        self.date = '011219'

        self.res = kwargs.get('res', (1024, 1280))
        self._exposure_time = kwargs.get('exposure_time', 0.05)
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

class BosonCamera(Boson):
    def __init__(self, port=None, device_id=None):
        self.port = port
        super().__init__(port=self.port)
        self.frame = None
        self.serial_no = str(self.get_camera_serial())
        # self.set_ffc_manual()
        self.set_ffc_auto()
        print("FFC Mode:", self.get_ffc_mode())
        if not self.get_gao_ffc_mode():
            print("The ffc correction is not being applied.")
        if not self.get_ffc_mode() == 0:
            print("The IR cameras need to be run in manual FFC mode.")
        self.device_id = self.find_video_device(device_id=device_id)

    def get_frame(self):
        return self.frame

    def update_frame(self):
        """
        Grab a frame from the camera and store it as a uint16.
        """
        self.frame = self.grab(device_id=self.device_id)
