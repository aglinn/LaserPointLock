from Packages.Cameras.Camera import Camera
from Packages.Errors import DeviceNotFoundError
import numpy as np
import re
from typing import List, Dict, Set, Tuple
import ctypes
import platform


class TProcessedDataProperty(ctypes.Structure):
    _fields_ = [("CameraID", ctypes.c_int),
                ("Row", ctypes.c_int),
                ("Column", ctypes.c_int),
                ("Bin", ctypes.c_int),
                ("XStart", ctypes.c_int),
                ("YStart", ctypes.c_int),
                ("ExposureTime", ctypes.c_int),
                ("RedGain", ctypes.c_int),
                ("GreenGain", ctypes.c_int),
                ("BlueGain", ctypes.c_int),
                ("TimeStamp", ctypes.c_int),
                ("TriggerOccured", ctypes.c_int),
                ("TriggerEventCount", ctypes.c_int),
                ("ProcessFrameType", ctypes.c_int)]


class MightexCameraAPI:
    GRAB_FRAME_FOREVER = 34952

    def __init__(self):
        # Attempt loading Mightex driver
        self.load_driver()

        # Initialize the API
        self.num_devices = self._initDevice()
        if self.num_devices <= 0:
            raise DeviceNotFoundError('No Mightex cameras were connected to the system')

        # Instantiate Mightex Camera instances for each connected camera
        self.device_list: List[MightexCamera] = []

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
        buffer_type = ctypes.c_char * 16
        regex = re.compile(r'[^a-zA-Z\-\d]')
        for i in range(self.num_devices):
            serial_no = buffer_type()
            module_no = buffer_type()
            self._getModuleNoSerialNo(i + 1, module_no, serial_no)
            serial_no = bytes(serial_no).decode('ascii')
            module_no = bytes(module_no).decode('ascii')
            serial_no = regex.sub('', serial_no)
            module_no = regex.sub('', module_no)
            self.module_no.append(module_no)
            self.serial_no.append(serial_no)
            self.dev_num[serial_no] = i + 1

            self.device_list.append(MightexCamera(self, serial_no, module_no))

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
        # self._startFrameGrab(self.GRAB_FRAME_FOREVER)
        # Install Callback Function
        self._installFrameHooker(ctypes.c_int(1), self.cGetFrameCallbackFuncPointer)

    def getDeviceList(self):
        return self.device_list

    def load_driver(self):
        try:
            if (platform.system() != "Windows"):
                raise OSError('This program requires a Windows system to interface with the Mightex cameras.')
            lib = ctypes.WinDLL(
                r'C:\Users\Kingdel\Documents\Mightex_SCX_CDROM_20190104\SDK\Lib\x64\NewClassic_USBCamera_SDK.dll')
        except FileNotFoundError:
            raise FileNotFoundError('Cannot use Mightex cameras without NewClassic_USBCamera_SDK.dll')

        c_ubyte_p = ctypes.POINTER(ctypes.c_ubyte)

        self._initDevice = lib['NewClassicUSB_InitDevice']
        self._addCameraToWorkingSet = lib['NewClassicUSB_AddDeviceToWorkingSet']
        self._removeDeviceFromWorkingSet = lib['NewClassicUSB_RemoveDeviceFromWorkingSet']
        self._startEngine = lib['NewClassicUSB_StartCameraEngine']
        self._stopEngine = lib['NewClassicUSB_StopCameraEngine']
        self._uninitDevice = lib['NewClassicUSB_UnInitDevice']
        self._setResolution = lib['NewClassicUSB_SetCustomizedResolution']
        self._startFrameGrab = lib['NewClassicUSB_StartFrameGrab']
        self._stopFrameGrab = lib['NewClassicUSB_StopFrameGrab']
        self._getModuleNoSerialNo = lib['NewClassicUSB_GetModuleNoSerialNo']
        self._getModuleNoSerialNo.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p]
        self._setExposureTime = lib['NewClassicUSB_SetExposureTime']

        def frameCallBack(attributes, frameptr):
            # print(attributes.XStart, attributes.YStart, attributes.ExposureTime, attributes.TimeStamp)
            # print(attributes.ProcessFrameType, attributes.TriggerOccured, attributes.TriggerEventCount)
            # print(attributes.Column)
            # create numpy array from RAW camera data
            if attributes and frameptr:
                self.attributes = attributes
                self.frame = self._frameFromFramePtr(frameptr)

                # _np_frame = np.ctypeslib.as_array(frameptr, shape=(3, attributes.Row, attributes.Column))
                # tframe = np.ones((attributes.Row, attributes.Column, 3))
                # img = cv2.imdecode(frameptr, cv2.IMREAD_ANYCOLOR)
                # # rgb = Image.frombytes('RGB', (attributes.Row, attributes.Column), tframe)
                # cv2.imshow('', img)

        self.cFuntionTypeWIthArgs = ctypes.CFUNCTYPE(None, TProcessedDataProperty, ctypes.c_char_p)
        self.cGetFrameCallbackFuncPointer = self.cFuntionTypeWIthArgs(frameCallBack)

        self._installFrameHooker = lib['NewClassicUSB_InstallFrameHooker']
        #                                   int FrameType,  FrameDataCallBack FrameHooker
        self._installFrameHooker.argtypes = [ctypes.c_int, self.cFuntionTypeWIthArgs]

        self._getFrame = lib['NewClassicUSB_GetCurrentFrame']
        self._getFrame.argtypes = [ctypes.c_int, ctypes.c_int, c_ubyte_p]
        self._getFrame.restype = c_ubyte_p

        ctypes.pythonapi.PyMemoryView_FromMemory.restype = ctypes.py_object

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

# Mightex Camera class
class MightexCamera(Camera):

    kind = "Mightex"

    def __init__(self, engine: MightexCameraAPI, serial_no: str, module_no: str):
        super().__init__()

        self.kind = "Mightex"
        self.engine: MightexEngine = engine
        if serial_no not in self.engine.serial_no:
            raise DeviceNotFoundError("Serial number {} is not connected to computer".format(serial_no))
        self.serial_no = serial_no
        self.frame = None
        self._exposure_time = 0.05
        # self.run_thread = CameraThread(self)
        # self.signals = CameraSignals()

    def get_frame(self, serial_no, n=1, getExposureTime=0, getTimeStamp=0):
        n = max(n, 1)  # if n < 1, set n = 1
        if serial_no not in self.active_devs:
            return None
        else:
            return self.frame

    def update_frame(self):
        #lock.lockForRead()
        # print("serial number just before calling the engine get frame function:",self.serial_no)
        self.frame, self.time = self.engine.get_frame(self.serial_no, 1, getTimeStamp=1)
        #lock.unlock()

    def set_exposure_time(self, time):
        # C code requires int in units of 50 us. 1 = 50 us. 10 = 500 us, etc; so convert ms to this unit.
        exposureTimeInt = int(time * 1000 / 50)
        self.engine._setExposureTime(self.serial_no, exposureTimeInt)
        self.frame = self.engine.get_frame(self.serial_no, 1, getExposureTime=1)
        self._exposure_time = self.engine._getExposure(self.serial_no)

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

    def terminate(self):
        pass

    @property
    def dev_num(self):
        return self.engine.dev_num[self.serial_no]            

    def _setExposure(self, serial_no, exposureTime):
        # C code requires int in units of 50 us. 1 = 50 us. 10 = 500 us, etc; so convert ms to this unit.
        exposureTimeInt = int(exposureTime*1000/50)
        self._setExposureTime(ctypes.c_int(self.dev_num[serial_no]), ctypes.c_int(exposureTimeInt))

    def _getExposure(self,serial_no):
        return self.exposureTime[serial_no]

    def _frameFromFramePtr(self, framePtr):
        # Get resolution from mightex engine
        res = self.resolution[serial_no]
        size = reduce(operator.mul, res, 1) + 56
        frame_type = ctypes.c_ubyte*size
        frame = frame_type()

        #lock.lockForWrite()
        print("serial number:",serial_no,"dev number:", self.dev_num[serial_no])
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

