import ctypes
import numpy as np
import re
import operator
import random
from functools import reduce
from typing import List, Dict, Set, Tuple
from abc import ABC, abstractmethod
from PyQt5 import QtCore
import time
from flirpy.camera.boson import Boson

lock = QtCore.QReadWriteLock()


class DeviceNotFoundError(Exception):
    pass

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

class BosonCamera(Camera):
    def __init__(self, port=None):
        self.frame = None
        self.port = port
        self.engine = Boson(port=self.port)
        self.serial_no = self.engine.get_camera_serial()
        self.set_ffc_manual()

    def get_frame(self):
        return self.frame

    def update_frame(self):
        """
        Grab a frame from the camera and store it as a uint16.
        """
        self.frame = self.engine.grab()

    def set_exposure_time(self, time):
        pass

    def set_resolution(self, res):
        pass

    def set_gain(self, gain):
        pass

    def set_decimation(self, gain):
        pass

    def set_ffc_manual(self):
        self.engine.set_ffc_manual()
        return

    def get_ffc_mode(self):
        """
        Get the current FFC mode

        0 = FLR_BOSON_NO_FFC_PERFORMED
        1 = FLR_BOSON_FFC_IMMINENT
        2 = FLR_BOSON_FFC_IN_PROGRESS
        3 = FLR_BOSON_FFC_COMPLETE
        4 = FLR_BOSON_FFCSTATUS_END

        Returns
        -------

            int
                FFC mode
        """
        return self.engine.get_ffc_mode()

    def set_ffc_auto(self):
        self.engine.set_ffc_auto()
        return

    def do_ffc(self):
        """
        Manually request a flat field correction (FFC)
        """
        self.engine.do_ffc()
        return

    def get_ffc_state(self):
        """
        Returns the FFC state:

        0 = FLR_BOSON_NO_FFC_PERFORMED
        1 = FLR_BOSON_FFC_IMMINENT
        2 = FLR_BOSON_FFC_IN_PROGRESS
        3 = FLR_BOSON_FFC_COMPLETE
        4 = FLR_BOSON_FFCSTATUS_END

        These return values are available as e.g.:

        flirpy.camera.boson.FLR_BOSON_NO_FFC_PERFORMED

        Returns
        -------

            int
                FFC state

        """
        return self.engine.get_ffc_state()

    def set_ffc_temperature_threshold(self, temp_change):
        """
        Set the change in camera temperature (Celsius) required before an FFC is requested.

        Parameters
        ----------
            float
                temperature change in Celsius
        """
        self.engine.set_ffc_temperature_threshold(temp_change)
        return

    def get_ffc_temperature_threshold(self):
        """
        Get the change in camera temperature before an FFC is requested

        Returns
        -------
            float
                temperature change in Celsius
        """
        return self.engine.get_ffc_temperature_threshold()

    def set_ffc_frame_threshold(self, seconds):
        """
        Set the number of seconds before an FFC is requested.

        Parameters
        ----------
            int
                seconds between FFC requests
        """
        self.engine.set_ffc_frame_threshold(seconds)
        return

    def get_ffc_frame_threshold(self):
        """
        Get the number of frames before an FFC is requested.
        """
        return self.engine.get_ffc_frame_threshold()

    def set_num_ffc_frame(self, num_frame):
        """
        Set the number of frames for the camera to integrate during FFC. 8 is factory default. With averager on, the
        time to perform ffc is doubled.
        :param num_frame: int 2, 4, 8, 16.
        :return:
        """
        self.engine.set_num_ffc_frame(num_frame)

        return

    def get_num_ffc_frame(self):
        """
        Get the number of frames the camera integrates during FFC.
        :return: int: 2,4,6, or 8
        """
        return self.engine.get_num_ffc_frame()

    def get_ffc_desired(self):
        """
        Get the state of the FFC desired flag. 0 = not desired. 1 = desired.
        :return: int 0 or 1
        """
        return self.engine.get_ffc_desired()

    def get_nuc_desired(self):
        """
        Get the state of the NUC table switch desired flag. 0 = not desired. 1 = desired.
        :return: int 0 or 1
        """
        return self.engine.get_nuc_desired()

    def do_nuc_table_switch(self):
        """
        This will perform a table NUC table switch, if the nuc table switch desired flag is set.
        :return:
        """
        self.engine.do_nuc_table_switch()

        return

    def get_fpa_temperature(self):
        """
        Get the current focal plane array (FPA) temperature in Celsius.

        Returns
        -------
            float
                FPA temperature in Celsius
        """
        return self.engine.get_fpa_temperature()

    @property
    def exposure_time(self):
        return 0.05

    @property
    def gain(self):
        return 1

    @property
    def dev_num(self):
        return self.engine.dev_num[self.serial_no]

    def get_averager_state(self):
        return self.engine.get_averager()

    def set_averager_state(self, value):
        self.engine.set_averager(value)
        return

    def set_pwr_on_defaults_factory(self):
        """
        This will restore power on defaults to factory settings. The camera should be disconnected and reconnected to
        make sure all changes take effect. A reboot does not appear to be enough.
        :return:
        """
        self.engine.set_pwr_on_defaults_factory()
        return

    def set_pwr_on_defaults(self):
        """
        Apply the current settings as power on defaults.
        :return:
        """
        self.engine.set_pwr_on_defaults()
        return

    def reboot(self):
        """
        Reboot the camera
        """
        self.engine.reboot()

        return

    def release(self):
        """
        This will release the camera hardware.
        :return:
        """
        self.engine.release()

        return

    def close(self):
        """
        Close the port.
        """
        self.engine.close()

        return

class MightexEngine:
    try:
        lib = ctypes.WinDLL(r'C:\Users\Kingdel\Documents\Mightex_SCX_CDROM_20190104\SDK\Lib\x64\NewClassic_USBCamera_SDK.dll')
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
    _setExposureTime = lib['NewClassicUSB_SetExposureTime']
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
        self.BufferPoint: Dict = {}  # This maps from serial number, string, to the pointer,
                                    # which is a pyvisa.ctwrapper.types.LP_c_ubyte object @ location
        self.buffer: Dict = {}  # This maps from serial number, string, to the buffer
        self.exposureTime: Dict = {}
        self.z: Dict = {}  # This is a Dict that maps serial number to the image from that camera.
        self.active_devs: Set[str] = set()
        buffer_type = ctypes.c_char*16
        regex = re.compile(r'[^a-zA-Z\-\d]')
        for i in range(self.num_devices):
            serial_no = buffer_type()
            module_no = buffer_type()
            self._getModuleNoSerialNo(i+1, module_no, serial_no)
            serial_no = bytes(serial_no).decode('ascii')
            module_no = bytes(module_no).decode('ascii')
            serial_no = regex.sub('', serial_no)
            module_no = regex.sub('', module_no)
            self.module_no.append(module_no)
            self.serial_no.append(serial_no)
            self.dev_num[serial_no] = i + 1
        print('Connected Cameras:')
        for ser, mod in zip(self.serial_no, self.module_no):
            print(ser, mod)
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

    def get_frame(self, serial_no, n=1, getExposureTime=0, getTimeStamp=0):
        n = max(n, 1)  # if n < 1, set n = 1
        if serial_no not in self.active_devs:
            return None
        else:
            res = self.resolution[serial_no]
            size = reduce(operator.mul, res, 1) + 56
            frame_type = ctypes.c_ubyte*size
            frame = frame_type()
            #lock.lockForWrite()
            # print("serial number:",serial_no,"dev number:", self.dev_num[serial_no])
            # print("dev num, just prior to get frame:",self.dev_num[serial_no])
            self.BufferPoint[serial_no] = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
            """
            # print("Device Number:",self.dev_num[serial_no], "SerialNumber:", serial_no,"Pointer:", self.BufferPoint[serial_no])
            # if self.BufferPoint[self.serial_no[0]] == self.BufferPoint[self.serial_no[1]]:
                # print("Both camera's buffer pointer point to the same space in memory.")
            #lock.unlock()
            #lock.lockForRead()
            # print("dev num, just prior to pymemoryview:",self.dev_num[serial_no])
            """
            self.buffer[serial_no] = ctypes.pythonapi.PyMemoryView_FromMemory(self.BufferPoint[serial_no], size)
            # print("dev num, just prior to copying from buffer:", self.dev_num[serial_no])
            self.z[serial_no] = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
            if (getTimeStamp+getExposureTime)>0:
                FrameInfo = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[24:44].copy()
                if getExposureTime:
                    self.exposureTime[serial_no] = int.from_bytes(FrameInfo[0:4], byteorder='little')*50.0/1000.0
                TimeStamp = int.from_bytes(FrameInfo[16:20], byteorder='little')
            #lock.unlock()
            for i in range(n - 1):
                #lock.lockForRead()
                self.BufferPoint[serial_no] = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
                self.buffer[serial_no] = ctypes.pythonapi.PyMemoryView_FromMemory(self.BufferPoint[serial_no], size)
                self.z[serial_no] += np.frombuffer(self.buffer, dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
                #lock.unlock()
            if n > 1:
                self.z[serial_no] = self.z[serial_no].astype(np.float) / n
            self.z[serial_no] = self.z[serial_no].astype(np.float)
            # ("dev num just prior to returning the frame", self.dev_num[serial_no])
            if getTimeStamp:
                return self.z[serial_no], TimeStamp
            else:
                return self.z[serial_no]

    def SetExposure(self, serial_no, exposureTime):
        # C code requires int in units of 50 us. 1 = 50 us. 10 = 500 us, etc; so convert ms to this unit.
        exposureTimeInt = int(exposureTime*1000/50)
        self._setExposureTime(ctypes.c_int(self.dev_num[serial_no]), ctypes.c_int(exposureTimeInt))

    def GetExposure(self,serial_no):
        return self.exposureTime[serial_no]

class MightexCamera(Camera):
    def __init__(self, engine: MightexEngine, serial_no: str):
        self.engine: MightexEngine = engine
        if serial_no not in self.engine.serial_no:
            raise DeviceNotFoundError("Serial number {} is not connected to computer".format(serial_no))
        self.serial_no = serial_no
        self.frame = None
        self._exposure_time = 0.05
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
        self.engine.SetExposure(self.serial_no, time)
        self.frame = self.engine.get_frame(self.serial_no, 1, getExposureTime=1)
        self._exposure_time = self.engine.GetExposure(self.serial_no)

    def set_resolution(self, res):
        self.engine._setResolution(self.engine.dev_num[self.serial_no], res[0], res[1], 0)

    def set_gain(self, gain):
        pass

    def set_decimation(self, gain):
        pass

    @property
    def exposure_time(self):
        return self._exposure_time

    @property
    def gain(self):
        return 1

    @property
    def dev_num(self):
        return self.engine.dev_num[self.serial_no]

