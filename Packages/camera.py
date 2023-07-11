import ctypes
import platform
import numpy as np
import re
import operator
import random
import cv2
from functools import reduce
from typing import List, Dict, Set, Tuple
from abc import ABC, abstractmethod
import PySpin
import EasyPySpin

class DeviceNotFoundError(Exception):
    pass

class MightexEngine:

    try:
        if (platform.system() != "Windows"):
            raise OSError('This program requires a Windows system to interface with the Mightex cameras.')
        folder_containing_Mightex_CDROM = r"C:\Users\Garri\OneDrive\Documents\Laser_Point_Lock_Software"
        lib = ctypes.WinDLL(folder_containing_Mightex_CDROM + r'\Mightex_SCX_CDROM_20190104\SDK\Lib\x64\NewClassic_USBCamera_SDK.dll')
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
    _setExposureTime = lib['NewClassicUSB_SetExposureTime']
    _setStartXY = lib['NewClassicUSB_SetXYStart']
    _setCameraWorkMode = lib['NewClassicUSB_SetCameraWorkMode']


    _getFrame = lib['NewClassicUSB_GetCurrentFrame']
    _getFrame.argtypes = [ctypes.c_int, ctypes.c_int, c_ubyte_p]
    _getFrame.restype = c_ubyte_p
    _SoftTrigger = lib['NewClassicUSB_SoftTrigger']

    ctypes.pythonapi.PyMemoryView_FromMemory.restype = ctypes.py_object

    def __init__(self, CameraWorkMode: list = [0,0,0,0,0,0]):
        """
        This finds and adds Mightex cameras to the engine for use in the camera code.
        CameraWorkMode is a list of integers to assign the work mode to each camera found.
        1 = capture images on trigger
        0 = capture images continuously.
        """
        # Initialize the API
        ret = self._initDevice()
        if ret <= 0:
            raise DeviceNotFoundError('No Mightex cameras were connected to the system')
        self.num_devices = ret
        # Find the module and serial numbers of connected cameras
        self.module_no: List[str] = []
        self.serial_no: List[str] = []
        self.dev_num: Dict[str, int] = {}
        self.dev_work_mode: Dict[str, int] = {}
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
        ret = self._startEngine(None, 8)
        if not ret == 1:
            raise Exception("Could not start the Mightex Engine.")

        for i in range(len(self.dev_num)):
            # Allow user to put camera into mode where frames are captured on trigger!
            self.dev_work_mode[self.serial_no[i]] = CameraWorkMode[i]
            if CameraWorkMode[i] == 1:
                ret = self._setCameraWorkMode(ctypes.c_int(self.dev_num[serial_no]),
                                              ctypes.c_int(self.dev_work_mode[self.serial_no[i]]))
                if not ret == 1:
                    raise Exception("Camera with serial number, ", serial_no, ", did not correctly set working mode"
                                                                              "to trigger mode.")

        # Set the default resolution to 1280x1024
        for i in range(self.num_devices):
            ret = self._setResolution(ctypes.c_int(i + 1), ctypes.c_int(1280), ctypes.c_int(1024), ctypes.c_int(0))
            if not ret == 1:
                raise Exception("Resolution not set for device with id, ", i+1)
        for i in range(len(self.dev_work_mode)):
            if self.dev_work_mode[self.serial_no[i]] == 0:
                # This seems to apply for both devices... not sure what to say about that?
                ret = self._startFrameGrab(self.GRAB_FRAME_FOREVER)
                if not ret == 1:
                    raise Exception("Did not start the Frame grab.")
        for i in range(len(self.dev_num)):
            img, time_stamp = self.get_frame(self.serial_no[i], getTimeStamp=1)
            print(time_stamp)
        return

    def activate_cam(self, serial_no):
        if serial_no not in self.serial_no:
            raise DeviceNotFoundError("Camera with serial number {} is not connected".format(serial_no))
        else:
            if serial_no not in self.active_devs:
                ret = self._addCameraToWorkingSet(self.dev_num[serial_no])
                if ret == 1:
                    self.active_devs.add(serial_no)

    def update_working_mode(self, working_mode, serial_no):
        ret = self._setCameraWorkMode(ctypes.c_int(self.dev_num[serial_no]), ctypes.c_int(working_mode))
        if not ret == 1:
            raise Exception("Camera with serial number, ", serial_no, ", did not correctly set working mode"
                                                                      "to trigger mode.")

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
            if self.dev_work_mode[serial_no] == 1:
                ret = self._SoftTrigger(ctypes.c_int(self.dev_num[serial_no]))
                if not ret == 1:
                    raise Exception("The frame capturing could not be triggered correctly for serial number, ",
                                    serial_no)
            res = self.resolution[serial_no]
            size = reduce(operator.mul, res, 1) + 56
            frame_type = ctypes.c_ubyte*size
            frame = frame_type()
            #lock.lockForWrite()
            # print("serial number:",serial_no,"dev number:", self.dev_num[serial_no])
            # print("dev num, just prior to get frame:",self.dev_num[serial_no])
            self.BufferPoint[serial_no] = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
            if self.BufferPoint[serial_no] is None:
                raise Exception("Camera Frame Grab failed for serial number, ", serial_no)
            """
            # print("Device Number:",self.dev_num[serial_no], "SerialNumber:", serial_no,"Pointer:", self.BufferPoint[serial_no])
            # if self.BufferPoint[self.serial_no[0]] == self.BufferPoint[self.serial_no[1]]:
                # print("Both camera's buffer pointer point to the same space in memory.")
            #lock.unlock()
            #lock.lockForRead()
            # print("dev num, just prior to pymemoryview:",self.dev_num[serial_no])
            """
            self.buffer[serial_no] = ctypes.pythonapi.PyMemoryView_FromMemory(self.BufferPoint[serial_no], int(size))
            # print("dev num, just prior to copying from buffer:", self.dev_num[serial_no])
            #self.z[serial_no] = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
            self.z[serial_no] = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[56:].reshape(
                res[::-1])
            if (getTimeStamp+getExposureTime) > 0:
                #FrameInfo = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[24:44].copy()
                FrameInfo = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[24:44]
                if getExposureTime:
                    self.exposureTime[serial_no] = int.from_bytes(FrameInfo[0:4], byteorder='little')*50.0/1000.0
                TimeStamp = int.from_bytes(FrameInfo[16:20], byteorder='little')
            #lock.unlock()
            for i in range(n - 1):
                #lock.lockForRead()
                self.BufferPoint[serial_no] = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
                self.buffer[serial_no] = ctypes.pythonapi.PyMemoryView_FromMemory(self.BufferPoint[serial_no], size)
                #self.z[serial_no] += np.frombuffer(self.buffer, dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
                self.z[serial_no] += np.frombuffer(self.buffer, dtype=np.uint8, count=size)[56:].reshape(
                    res[::-1])
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

    def GetExposure(self, serial_no):
        return self.exposureTime[serial_no]

    def update_resolution(self, serial_no, res):
        self.resolution[serial_no] = res

from PyQt5 import QtCore
from PyQt5.QtCore import QObject, QThread
import time
from flirpy.camera.boson import Boson
from flirpy.camera.core import Core
from PyQt5.QtCore import pyqtSlot, pyqtSignal


lock = QtCore.QReadWriteLock()


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


class MightexCamera(Camera):
    def __init__(self, engine: MightexEngine, serial_no: str):
        self.engine: MightexEngine = engine
        if serial_no not in self.engine.serial_no:
            raise DeviceNotFoundError("Serial number {} is not connected to computer".format(serial_no))
        self.serial_no = serial_no
        self.frame = None
        self._exposure_time = 0.05
        self._time = None
        self._ROI_bounds = None
        self._xy = (0, 0)
        self._time_wrapper_count = 0
        # self.run_thread = CameraThread(self)
        # self.signals = CameraSignals()

    def get_frame(self):
        return self.frame

    def update_frame(self):
        #lock.lockForRead()
        # print("serial number just before calling the engine get frame function:",self.serial_no)
        self.frame, self.time = self.engine.get_frame(self.serial_no, 1, getTimeStamp=1)
        #lock.unlock()

    def set_exposure_time(self, time):
        self.engine.SetExposure(self.serial_no, time)
        self.frame = self.engine.get_frame(self.serial_no, 1, getExposureTime=1)
        self._exposure_time = self.engine.GetExposure(self.serial_no)

    def set_resolution(self, res):
        self.engine._setResolution(ctypes.c_int(self.engine.dev_num[self.serial_no]), ctypes.c_int(int(res[0])),
                                   ctypes.c_int(int(res[1])), ctypes.c_int(0))
        self.engine.update_resolution(self.serial_no, res=res)
        return

    def set_startXY(self, xy: list):
        self.engine._setStartXY(ctypes.c_int(self.engine.dev_num[self.serial_no]),
                                ctypes.c_int(int(xy[0])), ctypes.c_int(int(xy[1])))
        self._xy = xy
        return

    @property
    def startXY(self):
        return self._xy

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

    def close(self):
        self.engine.deactivate_cam(self.serial_no)
        return

    @property
    def dev_num(self):
        return self.engine.dev_num[self.serial_no]

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        if len(self.time > 0):
            if value < self.time[-1] - self._time_wrapper_count * (65535 + 1):
                self._time_wrapper_count += 1  # Convert to a monotonic timestamp
        self._time.append(self._time_wrapper_count * (65535 + 1) + value)
        self._time = value
        return

    def apply_ROI(self):
        width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
        width -= np.mod(width, 4)
        height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
        height -= np.mod(height, 4)
        self.set_resolution((height, width))
        x = int(np.round(self.ROI_bounds[2]))
        y = int(np.round(self.ROI_bounds[0]))
        self.set_startXY((x, y))
        return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @ROI_bounds.setter
    def ROI_bounds(self, roi: list):
        self._ROI_bounds = roi

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

    def apply_ROI(self):
        pass

    def time(self):
        return 0


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
        self._time = 0
        self._ROI_bounds = (0, 1024, 0, 1280)
        self.is_boson = True

    def get_frame(self):
        return self.frame[int(self.ROI_bounds[0]):int(self.ROI_bounds[1]+1),
               int(self.ROI_bounds[2]):int(self.ROI_bounds[3]+1)]

    def update_frame(self):
        """
        Grab a frame from the camera and store it as a uint16.
        """
        self.frame = self.grab(device_id=self.device_id)
        return

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        self._time = value
        return

    @property
    def startXY(self):
        return (self._ROI_bounds[2], self._ROI_bounds[0])

    def apply_ROI(self):
        pass

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @ROI_bounds.setter
    def ROI_bounds(self, roi: list):
        self._ROI_bounds = np.round(roi)

    def service(self):
        pass

    def setup_camera_for_image_acquisition(self):
        pass


class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2


class BaseCamera(QObject):
    """
    This class is meant to be the parent of any camera class, it has the necessary minnimum signals and slots, but all
    slots must be overwritten with the appropriate functions for the camera being implemented. Slots are left as they
    were written for the blackfly_S to provide a bit of a guide, but they should be overwritten.


    BaseCamera was written to essentially operate the Blackfly_S, but with an init function that was generic, which
    allows other cameras to inherit from BaseCamera, where other cameras may overwrite all methods of BaseCamera, but
    Blackfly_S by design only needs to add a bit to the init function to operate correctly. Also, this means updates to
    Blackfly_S Should be done through the BaseCamera.

    """
    # signals needed to run a camera object on its own thread.
    img_captured_signal = pyqtSignal(np.ndarray, float)
    exposure_updated_signal = pyqtSignal(float)
    gain_updated_signal = pyqtSignal(float)
    ROI_applied = pyqtSignal(bool)
    r0_updated_signal = pyqtSignal(np.ndarray)
    request_update_timer_interval_signal = pyqtSignal(float, float)
    updated_image_size = pyqtSignal(int,int)

    def __init__(self, dev_id: int):
        super().__init__()
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.updated_image_size.emit(frame_width, frame_height)
        # Assume the camera is ready to acquire by the end of init function, override if neccessary.
        self._ready_to_acquire = True
        # Need to appropriately set a serial number for the camera, which is used in the camera device list of GUI to
        # connect camera.
        self.serial_no = str(dev_id)
        # Offset of the pixel coordinates, (0,0) because full view is used.
        self._startXY = [0, 0]
        # Used to toggle close function between swapping  cameras and closing the whole app
        self._app_closing = False
        # Will need to overwrite framerate with appropriate number.
        self.frame_rate = 1
        self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
        self.timeout_time = int(self.timeout_time)
        self.starting = True
        self._ROI_bounds = None
        # Unused for Blackfly_S, but handle wrapping time stamps when necessary.
        self._time = 0
        self._time_offset = 0
        self.is_boson = False
        self._exposure_time = self.timeout_time
        return

    @pyqtSlot()
    def get_frame(self):
        """
        Must capture a new frame and the time_stamp in ms. Then, emit img_captured_signal(frame, time_stamp).
        """
        ret, frame, time_stamp = self.cap.read()
        if ret:
            # Timestamp in ns as int convert to ms as float int-->float automatic.
            time_stamp = time_stamp * 10 ** -6
            self.img_captured_signal.emit(frame, time_stamp)
        return

    @pyqtSlot(float)
    def set_exposure_time(self, t):
        """
        Set's the camera exposure time. Updates the expected time between frames (timeout_time) and emits
        request_update_timer_interval_signal with new timeout_time, which the update manager uses to update its timer
        interval. Finally, emits exposure_updated_signal with new exposure time so that GUI updates
        displayed exposure time.
        FYI, The update manager uses a timer to request a new  image to be captured to make sure that the
        cameras are throttled and do not overload the update manager. So, every timeout_time ms's the update manager
        requests a new frame, or if the update manager is too busy, they request a new image when they get a chance to
        check and at least timeout_time has passed.
        """
        t *= 1000  # Convert from ms to us
        self.cap.set(cv2.CAP_PROP_EXPOSURE, t)
        self._exposure_time = self.cap.get(cv2.CAP_PROP_EXPOSURE) / 1000.0  # Convert back to ms
        # Update the timer interval as needed.
        frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        if self.frame_rate != frame_rate or self.starting:
            self.starting = False
            self.frame_rate = frame_rate
            self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
            self.timeout_time = int(self.timeout_time)
            self.request_update_timer_interval_signal.emit(self.timeout_time, frame_rate)
        self.exposure_updated_signal.emit(self.exposure_time)
        return

    @pyqtSlot(float)
    def set_gain(self, gain):
        """
        Set the camera gain, and emit gain_updated_signal with new gain.
        """
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self._gain = self.cap.get(cv2.CAP_PROP_GAIN)
        self.gain_updated_signal.emit(self.gain)
        return

    @property
    def exposure_time(self):
        return self._exposure_time

    @property
    def gain(self):
        return self._gain

    def setup_camera_for_image_acquisition(self):
        """
        Set any parameters and/or perform any priming functions necessary to begin acquisition.
        """
        if not self._ready_to_acquire:
            pass
        return

    @pyqtSlot(bool)
    def stop_capturing(self, app_closing=False):
        """
        Called when the user is changing which camera is connected to a camview.
        """
        self._app_closing = app_closing
        # Calls close function with not closing flag, to tell camera thread to delete camera, which then triggers on
        # GUI thread the reconnect camera function.
        self.close()
        return

    @pyqtSlot()
    def close(self):
        """
        Handle closing the camera. 2 cases: 1) closing the app, where the GUI will delete the camera object after the
        camera thread is stopped, hence cannot use deleteLater. 2) just changing which camera is connected as cam1 or
        cam2, in which case, the camera thread is not closing, so use deleteLater. In both cases, close the capture
        resource, and/or release camera as instructed by camera guides.
        """
        self.cap.release()
        if not self._app_closing:
            # So far only calling like this to disconnect the camera in an effort to reconnect another. So, delete this
            # object, but do not delete the thread, which is what emitting cap_released signal does.
            self.deleteLater()
        return

    @property
    def t(self):
        """
        time that the image was acquired at in ms.
        """
        return self._time

    @t.setter
    def t(self, value):
        """
        Make sure that the timestamp of the camera is set as a monotonic function. So, you may need to be applying an
        offset if your timestamp is wrapping. For Blackfly_S That wrapping time is years... so, no need and this is
        unused.
        """
        self._time = self._time_offset + value
        return

    @pyqtSlot()
    def apply_ROI(self):
        """
        Allow the user to apply an ROI to your camera, no need to read pixels that fall significantly outside of the
        laser beam profile. Setting the ROI on your camera allows faster frame rates and quicker data processing of the
        image.

        Use the self.ROI_bounds that are set graphically in the GUI by the user to then apply that ROI on your actual
        camera however that is done. E.G. For Blackfly_S, the acquisition had to be stopped to apply an ROI, so do
        appropriate things as needed.
        Update the start_XY as necessary, so that the measured position is the same when there is no ROI and after you
        apply an ROI.
        Update the frame rate and the timeout time and let the UpdateManager know the new timeout time as necessary.
        FYI, The update manager uses a timer to request a new  image to be captured to make sure that the
        cameras are throttled and do not overload the update manager. So, every timeout_time ms's the update manager
        requests a new frame, or if the update manager is too busy, they request a new image when they get a chance to
        check and at least timeout_time has passed.
        """
        if self.ROI_bounds is not None:
            # Get parameters to apply to the ROI settings of Camera
            width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
            height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
            width -= (width - 8) % 4  # make this difference divisible by 4
            height -= (height - 6) % 2
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))
            x -= x % 4
            y -= y % 2
            # Must make changes while not acquiring images. So, end acquisition, apply changes, begin acquisition
            self.cap.cam.EndAcquisition()
            self._ready_to_acquire = False

            x_offset = self.cap.get_pyspin_value('OffsetX')
            y_offset = self.cap.get_pyspin_value('OffsetY')
            print(x, y, width, height)
            if (x_offset == 0 and x != 0) or \
                    (y_offset == 0 and y != 0):
                # Then, I am trying to apply an offset and there is no offset yet, so it is better to apply the widths
                # first, so that I can apply the appropriate offset. If full width/height, then offset can only be (0,0)
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)
                self.cap.set_pyspin_value('OffsetX', x)
                self.cap.set_pyspin_value('OffsetY', y)
            elif x_offset == x and y_offset == y:
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)
            elif (x_offset != 0 and x == 0) or (y_offset != 0 and y == 0):
                # Then, I need to apply the offsets first, so that I am allowed to set full width/height
                self.cap.set_pyspin_value('OffsetX', x)
                self.cap.set_pyspin_value('OffsetY', y)
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)

            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            # Restart Acquisition/resetup camera to acquire images:
            self.cap.cam.BeginAcquisition()
            self._ready_to_acquire = True
            self.ROI_applied.emit(True)
            frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
            if frame_rate != self.frame_rate:
                self.frame_rate = frame_rate
                self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
                self.timeout_time = int(self.timeout_time)
                self.request_update_timer_interval_signal.emit(self.timeout_time, frame_rate)
            frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.updated_image_size.emit(frame_width, frame_height)
        return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @pyqtSlot(list)
    def set_ROI_bounds(self, roi: list):
        """
        set the ROI Bounds from GUI request and then apply them.
        """
        self._ROI_bounds = roi
        self.apply_ROI()
        return

    @property
    def startXY(self):
        return self._startXY

    @startXY.setter
    def startXY(self, XY):
        """
        Update the start XY of the pixel coordinates. Then, emit r0_updated_signal so that GUI and Update Manager know
        how to interpret image data received.
        """
        self._startXY = [XY[0], XY[1]]
        self.r0_updated_signal.emit(np.asarray(self._startXY))
        return

    def ensure_full_view(self):
        """
        This function should just make sure that there is no ROI applied to the camera, i.e. the camera is operating
        with the largest frame size possible.
        """
        try:
            self.cap.cam.EndAcquisition()
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            self.cap.cam.BeginAcquisition()
            self.ROI_applied.emit(False)
        except:
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            self.ROI_applied.emit(False)
        return


class Blackfly_S(BaseCamera):
    """
    BaseCamera was written to essentially operate the Blackfly_S, but with an init function that was generic, which
    allows other cameras to inherit from BaseCamera, where other cameras may overwrite all methods of BaseCamera, but
    Blackfly_S by design only needs to add a bit to the init function to operate correctly. Also, this means updates to
    Blackfly_S Should be done through the BaseCamera.
    """

    def __init__(self, dev_id: int):
        """
        Construct the Blackfly_S camera object.
        """
        # Init the base camera. which inits QObject.
        super().__init__(dev_id)

        ########################################
        # Additional Init steps for Blackfly_S #
        ########################################
        # Open Capture object
        self.cap = EasyPySpin.VideoCapture(dev_id)
        if not self.cap.isOpened():
            print("Camera can't open\nexit")
            return -1
        # Set camera settings.
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
        self.cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
        self.cap.set_pyspin_value("GammaEnable", False)
        print("Gamma is ", self.cap.get(cv2.CAP_PROP_GAMMA))
        self._exposure_time = 'Auto'
        self.ROI_full_bounds = [0, 100000, 0, 100000]  # This is static and only ever called by the GUI thread.
        self._gain = 1
        # Get the current frame rate and use that to set the timeout time for UpdateManager's timer for grabbing the
        # next image. Must overwrite appropriately.
        self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        self.timeout_time = np.floor((1 / self.frame_rate)*1000) # in ms
        self.timeout_time = int(self.timeout_time)
        self.starting = True
        self._ROI_bounds = None
        # Setup camera to be full view rather than in ROI mode.
        self.ensure_full_view()
        return


class BlackflyS_EasyPySpin_QObject(QObject):
    # TODO: Delete this once I confirm that Blackfly_S works correctly.

    # signals needed to run Blackfly S camera object on its own thread.
    img_captured_signal = pyqtSignal(np.ndarray, float)
    exposure_updated_signal = pyqtSignal(float)
    gain_updated_signal = pyqtSignal(float)
    ROI_applied = pyqtSignal(bool)
    r0_updated_signal = pyqtSignal(np.ndarray)
    request_update_timer_interval_signal = pyqtSignal(float)

    def __init__(self, dev_id: int):
        super().__init__()
        self.cap = EasyPySpin.VideoCapture(dev_id)

        if not self.cap.isOpened():
            print("Camera can't open\nexit")
            return -1
        self.ensure_full_view()
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
        self.cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
        self._ready_to_acquire = True
        self.serial_no = str(dev_id)
        self._startXY = [0, 0]
        self.cap.set_pyspin_value("GammaEnable", False)
        print("Gamma is ", self.cap.get(cv2.CAP_PROP_GAMMA))
        self._exposure_time = 'Auto'
        self.ROI_full_bounds = [0, 100000, 0, 100000] # This is static and only ever called by the GUI thread.
        self._gain = 1
        self._keep_capturing = True
        self._app_closing = False
        self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        self.timeout_time = np.floor((1 / self.frame_rate)*1000) # in ms
        self.timeout_time = int(self.timeout_time)
        self.starting = True
        self._ROI_bounds = None
        return

    @pyqtSlot()
    def get_frame(self):
        ret, frame, time_stamp = self.cap.read()
        if ret:
            # Timestamp in ns as int convert to ms as float int-->floaot automatic.
            time_stamp = time_stamp*10**-6
            self.img_captured_signal.emit(frame, time_stamp)
        return

    @pyqtSlot(float)
    def set_exposure_time(self, t):
        """
        Not implemented because auto exposure loop on camera is probably better than manually setting.
        """
        t *= 1000  # Convert from ms to us
        self.cap.set(cv2.CAP_PROP_EXPOSURE, t)
        self._exposure_time = self.cap.get(cv2.CAP_PROP_EXPOSURE)/1000.0  # Convert back to ms
        # Update the timer interval as needed.
        frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        if self.frame_rate != frame_rate or self.starting:
            self.starting = False
            self.frame_rate = frame_rate
            self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
            self.timeout_time = int(self.timeout_time)
            self.request_update_timer_interval_signal.emit(self.timeout_time)
        self.exposure_updated_signal.emit(self.exposure_time)
        return

    def set_resolution(self, res):
        pass

    @pyqtSlot(float)
    def set_gain(self, gain):
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self._gain = self.cap.get(cv2.CAP_PROP_GAIN)
        self.gain_updated_signal.emit(self.gain)
        return

    def set_decimation(self, decimation):
        return

    @property
    def exposure_time(self):
        return self._exposure_time

    @property
    def gain(self):
        return self._gain

    @staticmethod
    def configure_trig(cam, CHOSEN_TRIGGER):
        """
            This function configures the camera to use a trigger. First, trigger mode is
            set to off in order to select the trigger source. Once the trigger source
            has been selected, trigger mode is then enabled, which has the camera
            capture only a single image upon the execution of the chosen trigger.

             :param cam: Camera to configure trigger for.
             :type cam: CameraPtr
             :return: True if successful, False otherwise.
             :rtype: bool
            """
        result = True

        print('*** CONFIGURING TRIGGER ***\n')

        print(
            'Note that if the application / user software triggers faster than frame time, the trigger may be dropped / skipped by the camera.\n')
        print(
            'If several frames are needed per trigger, a more reliable alternative for such case, is to use the multi-frame mode.\n\n')

        if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
            print('Software trigger chosen ...')
        elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
            print('Hardware trigger chose ...')

        try:
            # Ensure trigger mode off
            # The trigger must be disabled in order to configure whether the source
            # is software or hardware.
            nodemap = cam.GetNodeMap()
            node_trigger_mode = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
            if not PySpin.IsAvailable(node_trigger_mode) or not PySpin.IsReadable(node_trigger_mode):
                print('Unable to disable trigger mode (node retrieval). Aborting...')
                return False

            node_trigger_mode_off = node_trigger_mode.GetEntryByName('Off')
            if not PySpin.IsAvailable(node_trigger_mode_off) or not PySpin.IsReadable(node_trigger_mode_off):
                print('Unable to disable trigger mode (enum entry retrieval). Aborting...')
                return False

            node_trigger_mode.SetIntValue(node_trigger_mode_off.GetValue())

            print('Trigger mode disabled...')

            # Set TriggerSelector to FrameStart
            # For this example, the trigger selector should be set to frame start.
            # This is the default for most cameras.
            node_trigger_selector = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSelector'))
            if not PySpin.IsAvailable(node_trigger_selector) or not PySpin.IsWritable(node_trigger_selector):
                print('Unable to get trigger selector (node retrieval). Aborting...')
                return False

            node_trigger_selector_framestart = node_trigger_selector.GetEntryByName('FrameStart')
            if not PySpin.IsAvailable(node_trigger_selector_framestart) or not PySpin.IsReadable(
                    node_trigger_selector_framestart):
                print('Unable to set trigger selector (enum entry retrieval). Aborting...')
                return False
            node_trigger_selector.SetIntValue(node_trigger_selector_framestart.GetValue())

            print('Trigger selector set to frame start...')

            # Select trigger source
            # The trigger source must be set to hardware or software while trigger
            # mode is off.
            node_trigger_source = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
            if not PySpin.IsAvailable(node_trigger_source) or not PySpin.IsWritable(node_trigger_source):
                print('Unable to get trigger source (node retrieval). Aborting...')
                return False

            if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
                node_trigger_source_software = node_trigger_source.GetEntryByName('Software')
                if not PySpin.IsAvailable(node_trigger_source_software) or not PySpin.IsReadable(
                        node_trigger_source_software):
                    print('Unable to set trigger source (enum entry retrieval). Aborting...')
                    return False
                node_trigger_source.SetIntValue(node_trigger_source_software.GetValue())
                print('Trigger source set to software...')

            elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
                node_trigger_source_hardware = node_trigger_source.GetEntryByName('Line0')
                if not PySpin.IsAvailable(node_trigger_source_hardware) or not PySpin.IsReadable(
                        node_trigger_source_hardware):
                    print('Unable to set trigger source (enum entry retrieval). Aborting...')
                    return False
                node_trigger_source.SetIntValue(node_trigger_source_hardware.GetValue())
                print('Trigger source set to hardware...')

            # Turn trigger mode on
            # Once the appropriate trigger source has been set, turn trigger mode
            # on in order to retrieve images using the trigger.
            node_trigger_mode_on = node_trigger_mode.GetEntryByName('On')
            if not PySpin.IsAvailable(node_trigger_mode_on) or not PySpin.IsReadable(node_trigger_mode_on):
                print('Unable to enable trigger mode (enum entry retrieval). Aborting...')
                return False

            node_trigger_mode.SetIntValue(node_trigger_mode_on.GetValue())
            print('Trigger mode turned back on...')

        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)
            return False

        return result

    def setup_camera_for_image_acquisition(self):

        if not self._ready_to_acquire:

            # Apply mono 8 pixel format
            #
            # *** NOTES ***
            # Enumeration nodes are slightly more complicated to set than other
            # nodes. This is because setting an enumeration node requires working
            # with two nodes instead of the usual one.
            #
            # As such, there are a number of steps to setting an enumeration node:
            # retrieve the enumeration node from the nodemap, retrieve the desired
            # entry node from the enumeration node, retrieve the integer value from
            # the entry node, and set the new value of the enumeration node with
            # the integer value from the entry node.
            #
            # Retrieve the enumeration node from the nodemap
            node_pixel_format = PySpin.CEnumerationPtr(self.nodemap.GetNode('PixelFormat'))
            if PySpin.IsAvailable(node_pixel_format) and PySpin.IsWritable(node_pixel_format):

                # Retrieve the desired entry node from the enumeration node
                node_pixel_format_mono8 = PySpin.CEnumEntryPtr(node_pixel_format.GetEntryByName('Mono8'))
                if PySpin.IsAvailable(node_pixel_format_mono8) and PySpin.IsReadable(node_pixel_format_mono8):

                    # Retrieve the integer value from the entry node
                    pixel_format_mono8 = node_pixel_format_mono8.GetValue()

                    # Set integer as new value for enumeration node
                    node_pixel_format.SetIntValue(pixel_format_mono8)

                    print('Pixel format set to %s...' % node_pixel_format.GetCurrentEntry().GetSymbolic())

                else:
                    print('Pixel format mono 8 not available...')

            else:
                print('Pixel format not available...')

            # Change bufferhandling mode to NewestOnly
            node_bufferhandling_mode = PySpin.CEnumerationPtr(self.sNodemap.GetNode('StreamBufferHandlingMode'))
            if not PySpin.IsAvailable(node_bufferhandling_mode) or not PySpin.IsWritable(node_bufferhandling_mode):
                print('Unable to set stream buffer handling mode.. Aborting...')
                return False

            # Retrieve entry node from enumeration node
            node_newestonly = node_bufferhandling_mode.GetEntryByName('NewestOnly')
            if not PySpin.IsAvailable(node_newestonly) or not PySpin.IsReadable(node_newestonly):
                print('Unable to set stream buffer handling mode.. Aborting...')
                return False

            # Retrieve integer value from entry node
            node_newestonly_mode = node_newestonly.GetValue()

            # Set integer value from entry node as new value of enumeration node
            node_bufferhandling_mode.SetIntValue(node_newestonly_mode)

            print('*** IMAGE ACQUISITION ***\n')
            try:
                node_acquisition_mode = PySpin.CEnumerationPtr(self.nodemap.GetNode('AcquisitionMode'))
                if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
                    print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
                    return False

                # Retrieve entry node from enumeration node
                node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
                if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(
                        node_acquisition_mode_continuous):
                    print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
                    return False

                # Retrieve integer value from entry node
                acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

                # Set integer value from entry node as new value of enumeration node
                node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

                print('Acquisition mode set to continuous...')

                #  Begin acquiring images
                #
                #  *** NOTES ***
                #  What happens when the camera begins acquiring images depends on the
                #  acquisition mode. Single frame captures only a single image, multi
                #  frame catures a set number of images, and continuous captures a
                #  continuous stream of images.
                #
                #  *** LATER ***
                #  Image acquisition must be ended when no more images are needed.
                self.cam.BeginAcquisition()

                self._ready_to_acquire = True

                print('Acquiring images...')
                return
            except PySpin.SpinnakerException as ex:
                print('Error: %s' % ex)
                return

    def update_settings(self):
        pass

    @pyqtSlot(bool)
    def stop_capturing(self, app_closing=False):
        # end the infinite capture loop by telling camera to not keep capturing
        # TODO: Tell timer to stop.
        # return to event loop, and upon next grab_frame emit signal to release cap.
        self._app_closing = app_closing
        return

    @pyqtSlot()
    def close(self):
        self.cap.release()
        if not self._app_closing:
            # So far only calling like this to disconnect the camera in an effort to reconnect another. So, delete this
            # object, but do not delete the thread, which is what emitting cap_released signal does.
            self.deleteLater()
        return

    @property
    def t(self):
        """
        time that the image was acquired at in ms.
        """
        return self._time

    @t.setter
    def t(self, value):
        self._time = self._time_offset + value
        return

    @pyqtSlot()
    def apply_ROI(self):
        if self.ROI_bounds is not None:
            # Get parameters to apply to the ROI settings of Camera
            width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
            height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
            width -= (width - 8) % 4  # make this difference divisible by 4
            height -= (height - 6) % 2
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))
            x -= x % 4
            y -= y % 2
            # Must make changes while not acquiring images. So, end acquisition, apply changes, begin acquisition
            self.cap.cam.EndAcquisition()
            self._ready_to_acquire = False

            x_offset = self.cap.get_pyspin_value('OffsetX')
            y_offset = self.cap.get_pyspin_value('OffsetY')
            print(x, y, width, height)
            if (x_offset == 0 and x != 0) or \
                    (y_offset == 0 and y != 0):
                # Then, I am trying to apply an offset and there is no offset yet, so it is better to apply the widths
                # first, so that I can apply the appropriate offset. If full width/height, then offset can only be (0,0)
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)
                self.cap.set_pyspin_value('OffsetX', x)
                self.cap.set_pyspin_value('OffsetY', y)
            elif x_offset == x and y_offset == y:
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)
            elif (x_offset != 0 and x == 0) or (y_offset != 0 and y == 0):
                # Then, I need to apply the offsets first, so that I am allowed to set full width/height
                self.cap.set_pyspin_value('OffsetX', x)
                self.cap.set_pyspin_value('OffsetY', y)
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)

            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            # Restart Acquisition/resetup camera to acquire images:
            self.cap.cam.BeginAcquisition()
            self._ready_to_acquire = True
            self.ROI_applied.emit(True)
            frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
            if frame_rate != self.frame_rate:
                self.frame_rate = frame_rate
                self.timeout_time = np.floor((1 / self.frame_rate)*1000)  # in ms
                self.timeout_time = int(self.timeout_time)
                self.request_update_timer_interval_signal.emit(self.timeout_time)
        return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @pyqtSlot(list)
    def set_ROI_bounds(self, roi: list):
        self._ROI_bounds = roi
        self.apply_ROI()
        return

    @property
    def startXY(self):
        return self._startXY

    @startXY.setter
    def startXY(self, XY):
        self._startXY = [XY[0], XY[1]]
        self.r0_updated_signal.emit(np.asarray(self._startXY))
        return

    def ensure_full_view(self):
        try:
            self.cap.cam.EndAcquisition()
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            self.cap.cam.BeginAcquisition()
            self.ROI_applied.emit(False)
        except:
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            self.ROI_applied.emit(False)
        return


class Boson_QObject(Boson, BaseCamera):
    # TODO: Can I set an ROI or not? No documentation that indicates that I can, but documentation says it is a fully
    # compliant USB video device. So, maybe that implies I can set an ROI using cv2?

    def __init__(self, port=None, device_id=None):
        # MRO: self, Boson, Core, BaseCamera, QObject, sip.wrapper, sip.simplewrapper, object
        # Init Boson class first.
        self.port = port
        super().__init__(port=self.port)
        serial_no = str(self.get_camera_serial())
        # With serial number in hand, can init BaseCamera Object
        super(Core, self).__init__(dev_id=serial_no)
        ##############################################################
        # Additional steps to finish initing the Boson_QObject class #
        ##############################################################
        self.frame = None
        # self.set_ffc_manual()
        self.set_ffc_auto()
        print("FFC Mode:", self.get_ffc_mode())
        if not self.get_gao_ffc_mode():
            print("The ffc correction is not being applied.")
        if not self.get_ffc_mode() == 0:
            print("The IR cameras need to be run in manual FFC mode.")
        # Find device by device_id
        self.device_id = self.find_video_device(device_id=device_id)
        # Setup the video device as a cv2 capture device.
        self.setup_video(self.device_id)
        self.set_ROI_bounds([0, 1024, 0, 1280])
        self.is_boson = True
        # Will need to overwrite framerate with appropriate number.
        # Cannot read Boson frame rate
        # self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        self.timeout_time = np.floor((1 / 60.0) * 1000)  # in ms
        self.timeout_time = int(self.timeout_time)
        self._exposure_time = self.timeout_time  # uniqute to Boson that does not have a settable exposure time.
        # TODO: Make sure that timer interval is updated with correct signal.
        # Unused for Blackfly_S, but handle wrapping time stamps when necessary.
        self._time_offset = time.monotonic()*1000 # in ms.
        self._time = self._time_offset
        return


    def get_frame(self):
        """
        Must capture a new frame and the time_stamp in ms. Then, emit img_captured_signal(frame, time_stamp).
        Boson does not provide the timestamp of the image, sadly. So, I must use a manual timestamp, hence the use of
        time.monotonic().
        """
        self.update_frame()
        self.t = time.monotonic()*1000  # in ms
        # Boson does not allow me to apply an ROI to the camera such that only those pixels are read. Thus, I apply an
        # ROI only manually as a means of croping the data after it is read. This does not get a speedup in FPS, but it
        # does still improve processing time of the images and importantly allows clipping unwanted data (bright pixels
        # by the edge of the detector.
        self.img_captured_signal.emit(self.frame[int(self.ROI_bounds[0]):int(self.ROI_bounds[1] + 1),
               int(self.ROI_bounds[2]):int(self.ROI_bounds[3] + 1)], self.t)
        return

    def update_frame(self):
        """
        Grab a frame from the camera and store it as a uint16.
        """
        self.frame = self.grab(device_id=self.device_id)
        return

    @property
    def t(self):
        return self._time

    @t.setter
    def t(self, value):
        """
        had to use a software timestamp :( So, using time.monotonic and want to subtract the initial time at initing the
        camera object.
        """
        self._time = value - self._time_offset
        return

    @pyqtSlot(list)
    def set_ROI_bounds(self, roi: list):
        """
        set the ROI Bounds from GUI request and then apply them.
        """
        self._ROI_bounds = np.round(roi)
        self.apply_ROI()
        return

    def apply_ROI(self):
        """
        Unfortunately, I cannot apply an ROI on the camera hardware to avoid waisting time reading unwanted pixels. So,
        this function will be different than for Blackfly_S.

        Just make sure that the startXY or image origin in pixel coordinates is updated and that other objects know what
        startXY is..
        """
        if self.ROI_bounds is not None:
            # Only need to update the startXY of the image that camera will be emitting.
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))

            self.startXY = [x, y]
            self.ROI_applied.emit(True)
        return

    def service(self):
        pass

    def ensure_full_view(self):
        """
        I am unable to apply an ROI on the actual camera, so camera will always return full view. i.e. never apply an
        ROI in the first place.
        """
        pass

    @pyqtSlot()
    def close(self):
        """
        Handle closing the camera. 2 cases: 1) closing the app, where the GUI will delete the camera object after the
        camera thread is stopped, hence cannot use deleteLater. 2) just changing which camera is connected as cam1 or
        cam2, in which case, the camera thread is not closing, so use deleteLater. In both cases, close the capture
        resource, and/or release camera as instructed by camera guides.
        """
        self.release()
        if not self._app_closing:
            # So far only calling like this to disconnect the camera in an effort to reconnect another. So, delete this
            # object, but do not delete the thread, which is what emitting cap_released signal does.
            self.deleteLater()
        return

    @pyqtSlot(float)
    def set_exposure_time(self, t):
        """
        Boson's do not have an exposure time. But, I am going to call this function on initializing the camera to
        tell the updatemanager to start the timer for grabbing frames, and also to tell the Update Manager what the
        "exposure time" is, since this is used in I term of PID. "exposure time" will just be considered 1/fps in ms or
        the same as timeout_time.
        """
        # both exposure time and timeout time are set in the init function and never change.
        self.request_update_timer_interval_signal.emit(self.timeout_time)
        self.exposure_updated_signal.emit(self.exposure_time)
        # TODO: Can I read exposure? Thus could I set an exposure time?
        print("Reading the Boson exposure time as ", self.cap.get(cv2.CAP_PROP_EXPOSURE))
        return

    @pyqtSlot(float)
    def set_gain(self, gain):
        """
        Set the camera gain, and emit gain_updated_signal with new gain.
        """
        # TODO: Can I use Gain like this or not with the BOSON?
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self._gain = self.cap.get(cv2.CAP_PROP_GAIN)
        self.gain_updated_signal.emit(self.gain)
        return