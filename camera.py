import ctypes
import numpy as np
import re
import operator
import random
from functools import reduce
from typing import List, Dict, Set, Tuple
from abc import ABC, abstractmethod


class DeviceNotFoundError(Exception):
    pass


class MightexEngine:
    try:
        lib = ctypes.WinDLL(r'C:\Users\kgord\Downloads\Mightex_SCX_CDROM_20190104\SDK\Lib\x64\NewClassic_USBCamera_SDK.dll')
    except FileNotFoundError:
        raise FileNotFoundError('Cannot use Mightex cameras without NewClassic_USBCamera_SDK.dll')
    GRAB_FRAME_FOREVER = 34952

    c_ubyte_p = ctypes.POINTER(ctypes.c_ubyte)

    _initDevice = lib['NewClassicUSB_InitDevice']
    _addCameraToWorkingSet = lib['NewClassicUSB_AddDeviceToWorkingSet']
    _removeDeviceFromWorkingSet = lib['NewClassicUSB_RemoveDeviceFromWorkingSet']
    _startEngine = lib['NewClassicUSB_StartCameraEngine']
    _stopEngine = lib['NewClassicUSB_StopCameraEngine']
    _uninitDevice = lib['NewClassicUSB_UnInitDevice']
    _setResolution = lib['NewClassicUSB_SetCustomizedResolution']
    _startFrameGrab = lib['NewClassicUSB_StartFrameGrab']
    _stopFrameGrab = lib['NewClassicUSB_StopFrameGrab']
    _getModuleNoSerialNo = lib['NewClassicUSB_GetModuleNoSerialNo']
    _getModuleNoSerialNo.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p]

    _getFrame = lib['NewClassicUSB_GetCurrentFrame']
    _getFrame.argtypes = [ctypes.c_int, ctypes.c_int, c_ubyte_p]
    _getFrame.restype = c_ubyte_p

    ctypes.pythonapi.PyMemoryView_FromMemory.restype = ctypes.py_object

    def __init__(self):
        # Initialize the API
        ret = self._initDevice()
        if ret <= 0:
            raise DeviceNotFoundError('No Mightex cameras were connected to the system')
        self.num_devices = ret
        # Find the module and serial numbers of connected cameras
        self.module_no: List[str] = []
        self.serial_no: List[str] = []
        self.dev_num: Dict[str, int] = {}
        self.active_devs: Set[str] = set()
        buffer_type = ctypes.c_char*16
        serial_no = buffer_type()
        module_no = buffer_type()
        regex = re.compile(r'[^a-zA-Z\-\d]')
        for i in range(self.num_devices):
            self._getModuleNoSerialNo(i + 1, module_no, serial_no)
            module_no = bytes(module_no).decode('ascii')
            serial_no = bytes(serial_no).decode('ascii')
            module_no = regex.sub('', module_no)
            serial_no = regex.sub('', serial_no)
            self.module_no.append(module_no)
            self.serial_no.append(serial_no)
            self.dev_num[serial_no] = i + 1
        # Default to 1280x1024 resolution
        self.resolution: Dict[str, Tuple[int]] = {s: (1280, 1024) for s in self.serial_no}
        # Activate at most the first two cameras
        if len(self.serial_no) > 0:
            self.activate_cam(self.serial_no[0])
            if len(self.serial_no) > 1:
                self.activate_cam(self.serial_no[1])
        # Start the camera engine
        self._startEngine(None, 8)
        # Set the default resolution to 1280x1024
        for i in range(self.num_devices):
            self._setResolution(ctypes.c_int(i + 1), ctypes.c_int(1280), ctypes.c_int(1024), ctypes.c_int(0))
        self._startFrameGrab(self.GRAB_FRAME_FOREVER)

    def activate_cam(self, serial_no):
        if serial_no not in self.serial_no:
            raise DeviceNotFoundError("Camera with serial number {} is not connected".format(serial_no))
        else:
            if serial_no not in self.active_devs:
                ret = self._addCameraToWorkingSet(self.dev_num[serial_no])
                if ret == 1:
                    self.active_devs.add(serial_no)

    def deactivate_cam(self, serial_no):
        if serial_no not in self.serial_no:
            raise DeviceNotFoundError("Camera with serial number {} is not connected".format(serial_no))
        else:
            if serial_no in self.active_devs:
                ret = self._removeDeviceFromWorkingSet(self.dev_num[serial_no])
                if ret == 1:
                    self.active_devs.remove(serial_no)

    def get_frame(self, serial_no, n=1):
        n = max(n, 1)  # if n < 1, set n = 1
        if serial_no not in self.active_devs:
            return None
        else:
            res = self.resolution[serial_no]
            size = reduce(operator.mul, res, 1) + 56
            frame_type = ctypes.c_ubyte*size
            frame = frame_type()
            x = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
            buffer = ctypes.pythonapi.PyMemoryView_FromMemory(x, size)
            z = np.frombuffer(buffer, dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1]).astype(np.uint)
            for i in range(n - 1):
                x = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
                buffer = ctypes.pythonapi.PyMemoryView_FromMemory(x, size)
                z += np.frombuffer(buffer, dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
            if n > 1:
                z = z / n
            z = z.astype(np.uint8)
            return z


class Camera(ABC):
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


class MightexCamera(Camera):
    def __init__(self, engine: MightexEngine, serial_no: str):
        self.engine: MightexEngine = engine
        if serial_no not in self.engine.serial_no:
            raise DeviceNotFoundError("Serial number {} is not connected to computer".format(serial_no))
        self.serial_no = serial_no

    def get_frame(self, n=1):
        return self.engine.get_frame(self.serial_no, n)

    def set_exposure_time(self, time):
        pass

    def set_resolution(self, res):
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


class FakeCamera(Camera):
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
        self.module_no = 'SCE-B013-U'
        self.serial_no = kwargs.get('serial_no', '{0:02}-{1:06}'.format(random.randint(10, 99), random.randint(0, 999999)))
        self.date = '011219'

        self.res = kwargs.get('res', (1024, 1280))
        self._exposure_time = kwargs.get('exposure_time', 0.05)
        self._gain = kwargs.get('gain', 1)
        self.decimation = kwargs.get('decimation', 1)
        self.xcen = kwargs.get('xcen', 0)
        self.ycen = kwargs.get('ycen', 0)
        self.comm_time = 0
        """Amount of time it takes to pass one frame of data through USB communication.
        Updated when get_frame() is called."""
        self.xres = self.res[0]
        self.yres = self.res[1]
        self.set_resolution((self.res[0], self.res[1]))
        self.set_exposure_time(self.exposure_time)
        self._grid = np.meshgrid((np.arange(self.xres) - self.xres/2)//self.decimation,
                                 (np.arange(self.yres) - self.yres/2)//self.decimation, indexing='ij')
    
    def set_resolution(self, res):
        self.xres, self.yres = res
        self._grid = np.meshgrid((np.arange(self.xres) - self.xres/2),
                                 (np.arange(self.yres) - self.yres/2), indexing='ij')

    @property
    def exposure_time(self):
        return self._exposure_time

    @property
    def gain(self):
        return self._gain

    def set_exposure_time(self, time):
        self._exposure_time = time
    
    def set_gain(self, gain):
        self._gain = gain*8
    
    def set_decimation(self, value):
        if isinstance(value, bool):
            self.decimation = 2 if value else 1
            self.set_resolution((self.xres, self.yres))
        else:
            raise ValueError("Decimation must be a boolean")
    
    def pixel_to_um(self, pixels):
        return pixels*5.5*self.decimation
    
    def get_frame(self, n=1):
        # Generate a new center
        self.xcen = min(self.xres//2, max(-self.xres//2, self.xcen + random.randint(-10, 10)))
        self.ycen = min(self.yres//2, max(-self.yres//2, self.ycen + random.randint(-10, 10)))
        # print('I am cam', self.serial_no, 'my xcen', self.xcen + self.xres//2, 'my ycen', self.ycen + self.yres//2)
        # Simulate a laser spot
        width = 0.02*self._grid[0].shape[0]
        image = np.round(245*np.exp(-((self._grid[0] - self.xcen)**2 + (self._grid[1] - self.ycen)**2)/(2*width**2))).astype(np.uint8)
        return image
