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
from PyQt5.QtCore import QObject
import time
from flirpy.camera.boson import Boson


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

class BlackflyS(Camera):

    def __init__(self, cam):
        self.cam = cam
        self.nodemap_tldevice = self.cam.GetTLDeviceNodeMap()
        self.cam.Init()
        self.nodemap = self.cam.GetNodeMap()
        self.sNodemap = self.cam.GetTLStreamNodeMap()
        #  Retrieve device serial number for device id
        #
        #  *** NOTES ***
        #  The device serial number is retrieved in order to keep cameras from
        #  overwriting one another. Grabbing image IDs could also accomplish
        #  this.
        self.serial_no = ''
        node_device_serial_number = PySpin.CStringPtr(self.nodemap_tldevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            self.serial_no = node_device_serial_number.GetValue()
            print('Device serial number retrieved as %s...' % self.serial_no)
        """"
        if not self.configure_trig(cam=self.cam, CHOSEN_TRIGGER = TriggerType.SOFTWARE):
            raise Exception('Trigger mode unable to be set')
        """

        # TODO: Enable setting ROI correctly. for now, just set startXY to 0,0
        self._startXY = [0, 0]
        # Init properties
        self.frame = None
        self._ready_to_acquire = False
        # TODO: Get the offset in time from the cameras time to a synchronized python timer correctly.
        self._time_offset = 0
        self._time = 0
        self._ROI_bounds = None
        return

    def update_frame(self):
        """
        Retrieve the next image, format appropriately, set frame property, and release the image.
        """
        try:
            #  Retrieve next received image
            #
            #  *** NOTES ***
            #  Capturing an image houses images on the camera buffer. Trying
            #  to capture an image that does not exist will hang the camera.
            #
            #  *** LATER ***
            #  Once an image from the buffer is saved and/or no longer
            #  needed, the image must be released in order to keep the
            #  buffer from filling up.

            image_result = self.cam.GetNextImage(1000)

            #  Ensure image completion
            if image_result.IsIncomplete():
                print('Image incomplete with image status %d ...' % image_result.GetImageStatus())
            else:
                # Getting the image data as a numpy array
                self.frame = image_result.GetNDArray()
                self.time = image_result.GetTimeStamp()/1e6 #time in ms

            #  Release image
            #
            #  *** NOTES ***
            #  Images retrieved directly from the camera (i.e. non-converted
            #  images) need to be released in order to keep from filling the
            #  buffer.
            image_result.Release()
            return
        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)

    def get_frame(self):
        return self.frame

    def set_exposure_time(self, time):
        """
        Not implemented because auto exposure loop on camera is probably better than manually setting.
        """
        pass
        return

    def set_resolution(self, res):
        pass

    def set_gain(self, gain):
        pass

    def set_decimation(self, decimation):
        return

    @property
    def exposure_time(self):
        return 0

    @property
    def gain(self):
        return 1

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

    def close(self):
        self.cam.EndAcquisition()
        self.cam.DeInit()
        del self.cam
        return

    @property
    def time(self):
        """
        time that the image was acquired at in ms.
        """
        return self._time

    @time.setter
    def time(self, value):
        self._time = self._time_offset + value
        return

    def apply_ROI(self):
        if self.ROI_bounds is not None:

            # Get parameters to apply to the ROI settings of Camera
            width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
            height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))

            # Must make changes while not acquiring images. So, end acquisition, apply changes, begin acquisition
            self.cam.EndAcquisition()
            self._ready_to_acquire = False

            # Apply offset X
            #
            # *** NOTES ***
            # Numeric nodes have both a minimum and maximum. A minimum is retrieved
            # with the method GetMin(). Sometimes it can be important to check
            # minimums to ensure that your desired value is within range.
            node_offset_x = PySpin.CIntegerPtr(self.nodemap.GetNode('OffsetX'))
            if PySpin.IsAvailable(node_offset_x) and PySpin.IsWritable(node_offset_x):

                min_x = node_offset_x.GetMin()
                max_x = node_offset_x.GetMax()
                if x < min_x:
                    print("applied min x offset")
                    node_offset_x.SetValue(min_x)
                elif x > max_x:
                    print("applied max x offset")
                    node_offset_x.SetValue(max_x)
                else:
                    print("applied my own offset")
                    node_offset_x.SetValue(ctypes.c_int(x))
            else:
                print('Offset X not available...')

            # Apply offset Y
            #
            # *** NOTES ***
            # Numeric nodes have both a minimum and maximum. A minimum is retrieved
            # with the method GetMin(). Sometimes it can be important to check
            # minimums to ensure that your desired value is within range.
            node_offset_y = PySpin.CIntegerPtr(self.nodemap.GetNode('OffsetY'))
            if PySpin.IsAvailable(node_offset_y) and PySpin.IsWritable(node_offset_y):

                min_y = node_offset_y.GetMin()
                max_y = node_offset_y.GetMax()
                if x < min_y:
                    node_offset_y.SetValue(min_y)
                elif x > max_y:
                    node_offset_x.SetValue(max_y)
                else:
                    node_offset_y.SetValue(ctypes.c_int(y))
            else:
                print('Offset Y not available...')

            # Set the width of the image
            node_width = PySpin.CIntegerPtr(self.nodemap.GetNode('Width'))
            if PySpin.IsAvailable(node_width) and PySpin.IsWritable(node_width):
                min_width = node_width.GetMin()
                max_width = node_width.GetMax()
                print(min_width)
                if width < min_width:
                    node_width.SetValue(min_width)
                elif width > max_width:
                    node_width.SetValue(max_width)
                else:
                    width = ctypes.c_int64(width)
                    node_width.SetValue(width, True)
            else:
                print("Width not available...")

            #Set the Height of the image
            node_height = PySpin.CIntegerPtr(self.nodemap.GetNode('Height'))
            if PySpin.IsAvailable(node_height) and PySpin.IsWritable(node_height):
                min_height = node_height.GetMin()
                max_height = node_height.GetMax()
                if height < min_height:
                    node_height.SetValue(min_height)
                elif height > max_height:
                    node_height.SetValue(max_height)
                else:
                    node_height.SetValue(ctypes.c_uint64(height))


            self.startXY = [self.cam.OffsetX, self.cam.offsetY]
            # Restart Acquisition/resetup camera to acquire images:
            self.setup_camera_for_image_acquisition()
            return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @ROI_bounds.setter
    def ROI_bounds(self, roi: list):
        self._ROI_bounds = roi

    @property
    def startXY(self):
        return self._startXY

    @startXY.setter
    def startXY(self, XY):
        self.cam.OffsetX(ctypes.c_int(XY[0]))
        self.cam.OffsetY(ctypes.c_int(XY[1]))
        self._startXY = [self.cam.OffsetX, self.cam.OffsetY]
        return

class BlackflyS_EasyPySpin(Camera,EasyPySpin.VideoCapture):

    def __init__(self, dev_id: int):
        self.cap = EasyPySpin.VideoCapture(dev_id)

        if not self.cap.isOpened():
            print("Camera can't open\nexit")
            return -1
        self.ensure_full_view()
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_time to auto
        self.cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
        self._ready_to_acquire = True
        self.serial_no = str(dev_id)
        self._startXY = [0,0]
        self.cap.set_pyspin_value("GammaEnable", False)
        print("Gamma is ", self.cap.get(cv2.CAP_PROP_GAMMA))
        self._exposure_time = 'Auto'
        self._gain = 1
        return

    def update_frame(self):
        pass
        return

    def get_frame(self):
        ret, frame = self.cap.read()
        if ret:
            return frame
        return

    def set_exposure_time(self, time):
        """
        Not implemented because auto exposure loop on camera is probably better than manually setting.
        """
        time *= 1000 #Convert from ms to us
        self.cap.set(cv2.CAP_PROP_EXPOSURE, time)
        self._exposure_time = self.cap.get(cv2.CAP_PROP_EXPOSURE)/1000.0 # Convert back to ms
        return

    def set_resolution(self, res):
        pass

    def set_gain(self, gain):
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self._gain = self.cap.get(cv2.CAP_PROP_GAIN)
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

    def close(self):
        self.cap.release()
        return

    @property
    def time(self):
        """
        time that the image was acquired at in ms.
        """
        return self._time

    @time.setter
    def time(self, value):
        self._time = self._time_offset + value
        return

    def apply_ROI(self):
        if self.ROI_bounds is not None:

            # Get parameters to apply to the ROI settings of Camera
            width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
            height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))

            # Must make changes while not acquiring images. So, end acquisition, apply changes, begin acquisition
            self.cap.cam.EndAcquisition()
            self._ready_to_acquire = False

            print(x,y,width,height)
            self.cap.set_pyspin_value('OffsetX', x)
            self.cap.set_pyspin_value('OffsetY', y)
            self.cap.set_pyspin_value('Width', width)
            self.cap.set_pyspin_value('Height', height)

            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            # Restart Acquisition/resetup camera to acquire images:
            self.cap.cam.BeginAcquisition()
            self._ready_to_acquire = True
            return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @ROI_bounds.setter
    def ROI_bounds(self, roi: list):
        self._ROI_bounds = roi

    @property
    def startXY(self):
        return self._startXY

    @startXY.setter
    def startXY(self, XY):
        self._startXY = [XY[0], XY[1]]
        return

    def ensure_full_view(self):
        try:
            self.cap.cam.EndAcquisition()
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.cap.cam.BeginAcquisition()
        except:
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
        return

from PyQt5.QtCore import pyqtSlot, pyqtSignal
from PyQt5.QtCore import Qt, QTimer


class BlackflyS_EasyPySpin_QObject(QObject):
    # signals needed to run Blackfly S camera object on its own thread.
    img_captured_signal = pyqtSignal(np.ndarray, float)
    capture_img_signal = pyqtSignal()
    exposure_updated_signal = pyqtSignal(float)
    exposure_set_signal = pyqtSignal(float)
    gain_set_signal = pyqtSignal(float)
    gain_updated_signal = pyqtSignal(float)
    close_signal = pyqtSignal(bool)
    apply_ROI_signal = pyqtSignal()
    ROI_applied = pyqtSignal(bool)
    ROI_bounds_set_signal = pyqtSignal(list)
    ROI_bounds_updated_signal = pyqtSignal()
    release_cap_signal = pyqtSignal()
    r0_updated_signal = pyqtSignal(np.ndarray)
    ROI_bounds_set_full_view_signal = pyqtSignal()
    request_start_timer = pyqtSignal()

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
        self._startXY = [0,0]
        self.cap.set_pyspin_value("GammaEnable", False)
        print("Gamma is ", self.cap.get(cv2.CAP_PROP_GAMMA))
        self._exposure_time = 'Auto'
        self.ROI_full_bounds = [0, 100000, 0, 100000] # This is static and only ever called by the GUI thread.
        self._gain = 1
        self._keep_capturing = True
        self._app_closing = False
        self.timer = QTimer()
        self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        self.timeout_time = np.floor((1 / self.frame_rate)*1000) # in ms
        self.timeout_time = int(self.timeout_time)
        self.timer.setInterval(self.timeout_time)  # int in ms. Need to set this better based on current frame rate.
        self.timer.setSingleShot(False)
        return

    def connect_signals(self):
        """
        Connect all camera received signals to slots.
        """
        # I used to set a queued connection to grab frames continuously inside grab frames continuously. Now user timer.
        # self.capture_img_signal.connect(self.grab_frames_continuously, type=Qt.QueuedConnection)
        self.exposure_set_signal.connect(self.set_exposure_time)
        self.gain_set_signal.connect(self.set_gain)
        self.close_signal.connect(self.stop_capturing)
        self.ROI_bounds_updated_signal.connect(self.apply_ROI_signal)  # If new ROI region set, apply it immediately.
        self.ROI_bounds_set_signal.connect(lambda v: setattr(self, 'ROI_bounds', v))
        self.release_cap_signal.connect(self.close)
        self.ROI_bounds_set_full_view_signal.connect(self.ensure_full_view)
        self.timer.destroyed.connect(self.new_timer)
        self.timer.timeout.connect(self.get_frame)
        self.request_start_timer.connect(self.start_timer)
        return

    @pyqtSlot()
    def start_timer(self):
        self.timer.start()

    def update_frame(self):
        pass
        return

    @pyqtSlot()
    def get_frame(self):
        ret, frame, time_stamp = self.cap.read()
        if ret:
            # Timestamp in ns as int convert to ms as float int-->floaot automatic.
            time_stamp = time_stamp*10**-6
            self.img_captured_signal.emit(frame, time_stamp)
        return

    @pyqtSlot()
    def grab_frames_continuously(self):
        self.get_frame()
        if not self._keep_capturing:
            self.timer.stop()
            self.release_cap_signal.emit()
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
        if self.frame_rate != frame_rate:
            self.frame_rate = frame_rate
            self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
            self.timeout_time = int(self.timeout_time)
            if self.timer.isActive():
                print("Destroying timer.")
                self.timer.timeout.disconnect()
                self.timer.stop()
                self.timer.deleteLater()
            else:
                print("Timeout time set to ", self.timeout_time)
                self.timer.setInterval(self.timeout_time)
        self.exposure_updated_signal.emit(self.exposure_time)
        return

    @pyqtSlot()
    def new_timer(self):
        """
        Create a new timer object and start it.
        """
        print("New timer")
        print("Timeout time set to ", self.timeout_time)
        self.timer = QTimer()
        self.timer.setSingleShot(False)
        self.timer.setInterval(self.timeout_time)
        # Make sure no connections
        self.timer.timeout.disconnect()
        self.timer.destroyed.disconnect()
        # Now reconnect.
        self.timer.timeout.connect(self.get_frame)
        self.timer.destroyed.connect(self.new_timer)
        self.timer.start()
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
        self._keep_capturing = False
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
            self.timer.stop()
            # Get parameters to apply to the ROI settings of Camera
            width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
            height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))

            # Must make changes while not acquiring images. So, end acquisition, apply changes, begin acquisition
            self.cap.cam.EndAcquisition()
            self._ready_to_acquire = False

            print(x, y, width, height)
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
                self.timeout_time = np.floor((1 / self.frame_rate)*1000) # in ms
                self.timeout_time = int(self.timeout_time)
                self.timer.setInterval(self.timeout_time)
            self.timer.start()
        return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @ROI_bounds.setter
    def ROI_bounds(self, roi: list):
        self._ROI_bounds = roi
        self.ROI_bounds_updated_signal.emit()
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