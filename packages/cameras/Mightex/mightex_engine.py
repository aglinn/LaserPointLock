import platform
import numpy as np
import re
import operator
from functools import reduce
from typing import List, Dict, Set, Tuple
from packages.exceptions import DeviceNotFoundError
from ctypes import *
from ctypes.wintypes import MSG
from PyQt5.QtCore import pyqtSignal, QObject, pyqtSlot, QMutex
from PyQt5.Qt import Qt


class INFO(Structure):
    """I think the below is the structure for a different type of camera
    _fields_ = [ ( "CameraID", c_int ),
     ( "Row", c_int ),
     ( "Column", c_int ),
     ( "Bin", c_int ),
     ( "XStart", c_int ),
     ( "YStart", c_int ),
     ( "ExposureTime", c_int ),
     ( "RedGain", c_int ),
     ( "GreenGain", c_int ),
     ( "BlueGain", c_int ),
     ( "TimeStamp", c_int ),
     ( "TriggerOccurred", c_int ),
     ( "TriggerEventCount", c_int ),
     ( "UserMark", c_int ),
     ( "FrameTime", c_int ),
     ( "CCDFrequency", c_int ),
     ( "FrameProcessType", c_int ),
     ( "FilterAcceptForFile", c_int ) ]"""
    _fields_ = [("CameraID", c_int),
                ("Row", c_int),
                ("Column", c_int),
                ("Bin", c_int),
                ("XStart", c_int),
                ("YStart", c_int),
                ("ExposureTime", c_int),
                ("RedGain", c_int),
                ("GreenGain", c_int),
                ("BlueGain", c_int),
                ("TimeStamp", c_int),
                ("TriggerOccurred", c_int),
                ("TriggerEventCount", c_int),
                ("ProcessFrameType", c_int)]


class MightexEngineSignals(QObject):
    # in
    queue_grab = pyqtSignal()
    start = pyqtSignal()
    set_exposure = pyqtSignal(str, int)
    set_gain = pyqtSignal(str, float)
    activate_camera = pyqtSignal(str, int)
    stop_grabbing = pyqtSignal()
    apply_roi = pyqtSignal(list)
    request_engine_shutdown = pyqtSignal()

    # out
    updated_exposure_time = pyqtSignal(int, float)
    img_captured = pyqtSignal(int, np.ndarray, float)
    updated_xy = pyqtSignal(int, np.ndarray)
    request_update_timer_interval = pyqtSignal(int, float, float)
    ROI_applied = pyqtSignal(int, bool)
    updated_frame_size = pyqtSignal(int, int, int)
    engine_shutdown = pyqtSignal()

    def __init__(self):
        super().__init__()
        return


class MightexEngine(QObject):
    #  Load appropriate DLLs
    try:
        if (platform.system() != "Windows"):
            raise OSError('This program requires a Windows system to interface with the Mightex cameras.')
        libw = windll.user32
        GetMessage = libw.GetMessageA
        TranslateMessage = libw.TranslateMessage
        DispatchMessage = libw.DispatchMessageA
        lib = cdll.LoadLibrary(r"packages/cameras/Mightex/libx64/NewClassic_USBCamera_SDK.dll")
    except FileNotFoundError:
        raise FileNotFoundError('Cannot use Mightex cameras without NewClassic_USBCamera_SDK.dll')

    FRAMECALLBACK = CFUNCTYPE(None, POINTER(INFO), POINTER(c_ubyte * 1280 * 960))

    def __init__(self):
        """
        Use the Mightex Engine and API to handle all communication with Mightex Cameras.
        """
        super().__init__()
        self.signals = MightexEngineSignals()
        self.signals.setParent(self)
        # Initialize the API
        ret = self.self.lib.NewClassicUSB_InitDevice()
        if ret <= 0:
            raise DeviceNotFoundError('No Mightex cameras were connected to the system')
        self.num_devices = ret
        # Get module_no and serial_no of cameras connected to the computer, add to working set, deactivate cameras
        self.module_no: List[str] = []
        self.serial_no: List[str] = []
        # WARNING: WRITTEN ASSUMING THAT CAMERAID IS NEVER WRITTEN OUTSIDE OF THIS INIT FUNCTION other threads will read
        self.cameraID: Dict[str, int] = {}
        self._exposure_times: List[float] = [0, 0]  # in ms
        regex = re.compile(r'[^a-zA-Z\-\d]')
        for i in range(self.num_devices):
            module_no = create_string_buffer(16)
            serial_no = create_string_buffer(16)
            ret = self.lib.NewClassicUSB_GetModuleNoSerialNo(i + 1, module_no, serial_no)
            if not ret == 1:
                print("WARNING: Could not get module and serial numbers for camera with id {}.".format(i+1))
            serial_no = bytes(serial_no).decode('ascii')
            module_no = bytes(module_no).decode('ascii')
            serial_no = regex.sub('', serial_no)
            module_no = regex.sub('', module_no)
            self.module_no.append(module_no)
            self.serial_no.append(serial_no)
            self.cameraID[serial_no] = i + 1
            ret = self.lib.NewClassicUSB_AddDeviceToWorkingSet(self.cameraID[serial_no])
            if not ret == 1:
                print("WARNING: Camera with serial number {} was not added to the working set".format(serial_no))
            self._exposure_times[i] = 0

        # Init some variables
        self.framecb = None
        self.msg = None
        self.active = False
        self.locks = [QMutex(), QMutex]
        self.frames: List[np.ndarray, np.ndarray] = [None, None]
        self._t_frames: List[float, float] = [0, 0]
        self._time_wrapper_count: List[int, int] = [0, 0]
        self._xy: List[tuple, tuple] = [(0, 0), (0, 0)]
        self.update_manager_ready_for_frames: List[bool, bool] = [True, True]  # ensure update manager knows startXY
        self._frame_sizes: List[tuple, tuple] = [(1280, 1024), (1280, 1024)]
        self._fps: List[float, float] = [32, 32]
        self.starting: List[bool, bool] = [True, True]
        self.active_cameras: List[int, int] = []
        self._app_closing = False
        return

    def setup_mightex_engine_for_capture(self):
        self.start_engine()
        for serial_no in self.serial_no:
            # Deactivate all cameras. They will be activated as needed.
            ret = self.lib.NewClassicUSB_ActiveDeviceInWorkingSet(self.cameraID[serial_no], 0)
            if not ret == 1:
                print("WARNING: Could not deactivate the camera with serial number {}.".format(serial_no))
            # Ensure full view for all cameras on initial startup of engine:
            self.set_ROI(serial_no, [0, 1024, 0, 1280])
        # connect signals
        self.connect_signals()
        # Setup Frame Hooker
        self.framecb = self.getCallback()
        print("InstallFrameHooker =", self.lib.NewClassicUSB_InstallFrameHooker(0, self.framecb))
        return

    @pyqtSlot(str, int)
    def change_camera_active(self, serial_no, active):
        """
        Cameras must be in the working set, but they can be activated and deactived with this command.
        active = 1 activate camera, 0 deactivate camera
        """
        ret = self.lib.NewClassicUSB_ActiveDeviceInWorkingSet(self.cameraID[serial_no], active)
        if not ret == 1:
            print("WARNING: Camera with serial number {} active state in "
                  "working set could not be set to ".format(serial_no), str(active))
        else:
            if active == 0 and self.cameraID[serial_no] in self.active_cameras:
                ind = np.where(np.asarray(self.active_cameras) == self.cameraID[serial_no])
                del self.active_cameras[ind]
        return

    def start_engine(self):
        ret = self.lib.NewClassicUSB_StartCameraEngine(None, 8)
        if not ret == 1:
            print("WARNING: There are no cameras in the working set. Activate cameras before starting engine.")

    def connect_signals(self):
        self.signals.queue_grab.connect(self.grab, type=Qt.QueuedConnection)
        self.signals.set_exposure.connect(self.set_exposure)
        self.signals.set_gain.connect(self.set_gain)
        self.signals.activate_camera.connect(self.change_camera_active)
        self.signals.stop_grabbing.connect(self.stop_grabbing)
        self.signals.apply_roi.connect(self.set_ROI)
        self.signals.request_engine_shutdown.connect(self.shutdown)
        return

    @pyqtSlot(int)
    def xy_updated(self, camID):
        self.update_manager_ready_for_frames[camID-1] = True
        return

    def find_cameras(self):
        print("InitDevice =", self.lib.NewClassicUSB_InitDevice())  # Initialize device
        moduleno = create_string_buffer(16)
        serialno = create_string_buffer(16)
        print("GetModuleNoSerialNo = ", self.lib.NewClassicUSB_GetModuleNoSerialNo(1, moduleno, serialno))
        print("ModuleNo =", repr(moduleno.raw))  # Show model number
        print("SerialNo =", repr(serialno.raw))  # Show serial number
        return

    # Create frame callback function
    def getCallback(self):
        def FrameCallback(info, data):
            x = data.contents
            camID = info.contents.CameraID
            self.xy = (camID, (info.contents.XStart, info.contents.YStart))
            if self.update_manager_ready_for_frames[camID-1]:
                frame = np.array(x)
                self.locks[camID-1].lock()
                self.frames[camID-1] = frame
                self.t_frames = (camID, info.contents.TimeStamp)
                self.locks[camID-1].unlock()
                self.exposure_times = (camID, info.contents.ExposureTime)
                self.frame_sizes = (camID, (info.contents.Column, info.contents.Row))
                if camID not in self.active_cameras:
                    self.active_cameras.append(camID)
            else:
                self.locks[camID - 1].lock()
                self.frames[camID - 1] = None
                self.locks[camID - 1].unlock()
            return
        return self.FRAMECALLBACK(FrameCallback)

    @pyqtSlot()
    def start(self):
        """
        If frame grabbing is not active, then start frame grabbing.
        """
        if not self.active:
            ret = self.lib.NewClassicUSB_StartFrameGrab(0x8888)
            if not ret == 1:
                print("WARNING: Could not start frame grab.")
                return
            self.active = True
            self.msg = MSG()
            self.signals.queue_grab.emit()  # Start the frame grabbing.
        return

    @pyqtSlot()
    def grab(self):
        """
        This invokes the FrameCallback function that reads frames with the Frame Hooker method.
        """
        if self.active:
            if self.GetMessage(pointer(self.msg), 0, 0, 0):
                if self.msg.message == self.win32con.WM_TIMER:
                    self.TranslateMessage(pointer(self.msg))
                    self.DispatchMessage(pointer(self.msg))
            self.signals.queue_grab.emit()  # keep looping over this, but allow other signals to hit the queue.
        return

    @pyqtSlot()
    def shutdown(self):
        if not self.active_cameras or self._app_closing:
            print("StopFrameGrab =", self.lib.NewClassicUSB_StopFrameGrab(1))
            print("StopCameraEngine =", self.lib.NewClassicUSB_StopCameraEngine())
            print("UnInitDevice =", self.lib.NewClassicUSB_UnInitDevice())
            self.active = False
            self.signals.engine_shutdown.emit()
        return

    def un_init_device(self):
        ret = self.lib.NewClassicUSB_UnInitDevice()
        if not ret == 1:
            print("WARNING: Could not uninit the API for Mightex Engine.")
        return

    @pyqtSlot()
    def stop_grabbing(self):
        """
        Actually bad names, this will shutdown mightex engine
        """
        if not self.active_cameras:
            ret = self.lib.NewClassicUSB_StopFrameGrab(1)
            if not ret == 1:
                print("WARNING: Frame grab did not stop!")
            else:
                self.active = False
        return

    @pyqtSlot(str, float)
    def set_exposure(self, serial_no, exp_time):
        """
        Set the exposure time on camera with serial_no.
        exp_time units of ms.
        API requires that the exposure time be formatted as an int with units of 50 us, so 10=500us.
        """
        exp_int = int(exp_time * 1000 / 50.0)
        ret = self.lib.NewClassicUSB_SetExposureTime(self.cameraID[serial_no], exp_int)
        if not ret == 1:
            print("WARNING: Exposure time was not set for camera with serial number {}".format(serial_no))
        return

    @property
    def exposure_times(self):
        return self._exposure_times

    @exposure_times.setter
    def exposure_times(self, camID_exp: tuple):
        """
        camID_exp is a tuple with the camera id (int) in the first position and the exposure time in the second.
        """
        if self.exposure_times[camID_exp[0]-1] != camID_exp[1] or self.starting[camID_exp[0]-1]:
            self.starting[camID_exp[0] - 1] = False
            self._exposure_times[camID_exp[0]-1] = camID_exp[1]
            exp_time = 50*camID_exp[1]*1000  # convert to ms
            self.signals.updated_exposure_time.emit(camID_exp[0], exp_time)
        return

    @property
    def frame_sizes(self):
        return self._frame_sizes

    @frame_sizes.setter
    def frame_sizes(self, camID_size: tuple):
        """
        camID_exp is a tuple with the camera id (int) in the first position and the exposure time in the second.
        """
        if self.frame_sizes[camID_size[0] - 1] != camID_size[1]:
            self._frame_sizes[camID_size[0] - 1] = camID_size[1]
            width, height = camID_size[1]
            self.signals.updated_frame_size.emit(camID_size[0], width, height)
        return

    @property
    def xy(self):
        return self._xy

    @xy.setter
    def xy(self, camID_xy: tuple):
        """
        camID_exp is a tuple with the camera id (int) in the first position and the (x,y) (tuple) in the second.
        """
        if self.xy[camID_xy[0] - 1] != camID_xy[1]:
            self._xy[camID_xy[0] - 1] = camID_xy[1]
            self.updated_xy.emit(camID_xy[0], np.asarray(camID_xy[1]))
            self.update_manager_ready_for_frames[camID_xy[0]-1] = False
        return

    @pyqtSlot(str, list)
    def set_ROI(self, serial_no, roi):
        x0 = int(roi[2])
        y0 = int(roi[0])
        success = True
        if x0 == 0 and y0 == 0:
            # Then make full size
            ret = self.lib.NewClassicUSB_SetXYStart(self.cameraID[serial_no], x0, y0)
            if not ret == 1:
                print("WARNING: Could not set a new starting x,y on camera with serial number {}.".format(serial_no))
                success = False
            ret = self.lib.NewClassicUSB_SetCustomizedResolution(self.cameraID[serial_no], 1280, 1024, 0)
            if not ret == 1:
                print("WARNING: Could not set resolution on camera with serial number {}.".format(serial_no))
                success = False
            if success:
                self.signals.ROI_applied.emit(self.cameraID[serial_no], False)
            return
        width = int(np.round(roi[1] - roi[0]))  # number of columns
        width -= np.mod(width, 4)
        height = int(np.round(roi[3] - roi[2]))  # number of rows
        height -= np.mod(height, 4)
        ret = self.lib.NewClassicUSB_SetCustomizedResolution(self.cameraID[serial_no], height, width, 0)
        if not ret == 1:
            print("WARNING: Could not set resolution on camera with serial number {}.".format(serial_no))
            success = False
        ret = self.lib.NewClassicUSB_SetXYStart(self.cameraID[serial_no], x0, y0)
        if not ret == 1:
            print("WARNING: Could not set a new starting x,y on camera with serial number {}.".format(serial_no))
            success = False
        if success:
            self.signals.ROI_applied.emit(self.cameraID[serial_no], True)
        return

    @pyqtSlot(str, float)
    def set_gain(self, serial_no, gain):
        """
        Set the gain on the monochrome camera.
        API takes an int, (1-64) which converts to gain as (int)*8/64=gain
        Monochrome Cameras just take the green gain's value.
        """
        gain_int = int(gain*64/8)
        ret = self.NewClassicUSB_SetGains(self.cameraID[serial_no], gain_int, gain_int, gain_int)
        if not ret == 1:
            print("WARNING: Could not set gain on camera with serial number {}".format(serial_no))
            return
        return

    def send_frame(self, serial_no):
        """
        Send signal with the camID, frame, and time_stamp.
        """
        camID = self.cameraID[serial_no]
        self.locks[camID-1].lock()
        frame = self.frames[camID-1]
        if frame is None:
            self.locks[camID-1].unlock()
            return
        t = self.t_frames[camID-1]
        self.signals.img_captured.emit(camID, frame, t)
        self.frames[camID - 1] = None
        self.locks[camID-1].unlock()
        return

    @property
    def t_frames(self):
        return self._t_frames

    @t_frames.setter
    def t_frames(self, camID_t:tuple):
        """
        camID_t is a tuple with the camid as the first position (int) and the time stamp, t, in the second position.
        """
        # time in ms time automatically wraps, so unwrap the time to be monotonic
        if camID_t[1] < self.t_frames[camID_t[0]-1] - self._time_wrapper_count[camID_t[0]-1] * (65535 + 1):
            self._time_wrapper_count[camID_t[0]-1] += 1  # Convert to a monotonic timestamp
        t1 = self._time_wrapper_count * (65535 + 1) + camID_t[1]
        dt = t1 - self._t_frames[camID_t[0]-1]
        fps = 1000/dt
        self.fps = (camID_t[0], fps)
        self._t_frames[camID_t[0]-1] = t1
        return

    @property
    def fps(self):
        return self._fps

    @fps.setter
    def fps(self, camID_fps: tuple):
        """
        camID_fps is a tuple with the camera id (int) in the first position and the fps (float) in the second.
        """
        if np.abs(np.floor(self.fps[camID_fps[0] - 1] - camID_fps[1])) >= 1:
            self._fps[camID_fps[0] - 1] = camID_fps[1]
            timeout_time = np.floor((1 / camID_fps[1]) * 1000)  # in ms
            self.signals.request_update_timer_interval.emit(camID_fps[0], int(timeout_time), camID_fps[1])
        return

    def get_camID(self, serial_no: str):
        return self.cameraID[serial_no]