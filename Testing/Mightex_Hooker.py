import copy
import platform
import numpy as np
from ctypes import *
from ctypes.wintypes import MSG
from PyQt5.QtCore import QThread, pyqtSignal, QObject, pyqtSlot
from PyQt5.Qt import Qt
from PyQt5.QtWidgets import QApplication
import sys
import cv2
import time
import win32con


libc = cdll.msvcrt
kbhit = libc._kbhit
libw = windll.user32
GetMessage = libw.GetMessageA
TranslateMessage = libw.TranslateMessage
DispatchMessage = libw.DispatchMessageA


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


class MightexEngine(QObject):
    #  Load appropriate DLLs
    try:
        if (platform.system() != "Windows"):
            raise OSError('This program requires a Windows system to interface with the Mightex cameras.')
        libw = windll.user32
        GetMessage = libw.GetMessageA
        TranslateMessage = libw.TranslateMessage
        DispatchMessage = libw.DispatchMessageA

        libc = cdll.msvcrt
        kbhit = libc._kbhit
        lib = cdll.LoadLibrary(r"x64/NewClassic_USBCamera_SDK.dll")
    except FileNotFoundError:
        raise FileNotFoundError('Cannot use Mightex cameras without NewClassic_USBCamera_SDK.dll')

    FRAMECALLBACK = CFUNCTYPE(None, POINTER(INFO), POINTER(c_ubyte * 1280 * 960))
    frame_read = pyqtSignal(np.ndarray, POINTER(INFO))
    # frame_read = pyqtSignal(np.ndarray, INFO)
    queue_grab = pyqtSignal()
    start_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        print("InitDevice =", self.lib.NewClassicUSB_InitDevice())  # Initialize device
        moduleno = create_string_buffer(16)
        serialno = create_string_buffer(16)
        print("GetModuleNoSerialNo = ", self.lib.NewClassicUSB_GetModuleNoSerialNo(1, moduleno, serialno))
        print("ModuleNo =", repr(moduleno.raw))  # Show model number
        print("SerialNo =", repr(serialno.raw))  # Show serial number
        print("AddDeviceToWorkingSet =", self.lib.NewClassicUSB_AddDeviceToWorkingSet(1))
        print("StartCameraEngine =", self.lib.NewClassicUSB_StartCameraEngine(None, 8))

        self.framecb = self.getCallback()  # Create callback function
        print("InstallFrameHooker =", self.lib.NewClassicUSB_InstallFrameHooker(0, self.framecb))
        self.lib.NewClassicUSB_SetExposureTime(1, 200)  # Set exposure time to 10ms
        self.msg = None
        self.queue_grab.connect(self.grab, type=Qt.QueuedConnection)
        self.start_signal.connect(self.start)
        return

    # Create frame callback function
    def getCallback(self):
        def FrameCallback(info, data):
            x = data.contents
            arr = np.array(x)
            self.frame_read.emit(arr, info)
            # logger.finished(arr, info)
            return
        return self.FRAMECALLBACK(FrameCallback)

    @pyqtSlot()
    def start(self):
        print("")
        print("Select an option: ")
        print("1 - Live View")
        print("2 - Quit application")
        inp = input("Enter an option: ")

        if (inp == "1"):
            ret = self.lib.NewClassicUSB_StartFrameGrab(0x8888)
            print("StartFrameGrab =", ret)
            if ret == 1:
                self.active = True
                self.msg = MSG()
                self.queue_grab.emit()

    @pyqtSlot()
    def grab(self):
        if self.active:
            if GetMessage(pointer(self.msg), 0, 0, 0):
                if self.msg.message == win32con.WM_TIMER:
                    TranslateMessage(pointer(self.msg))
                    DispatchMessage(pointer(self.msg))
            self.queue_grab.emit()
        return

    @pyqtSlot()
    def stop(self):
        print("StopFrameGrab =", self.lib.NewClassicUSB_StopFrameGrab(1))
        print("StopCameraEngine =", self.lib.NewClassicUSB_StopCameraEngine())
        print("UnInitDevice =", self.lib.NewClassicUSB_UnInitDevice())
        self.active = False
        return


class Logger(QObject):
    finished_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.num_frames_acquired = 0
        return

    @pyqtSlot(np.ndarray, Structure)
    def finished(self, img, info):
        self.num_frames_acquired += 1
        handle = "camera " + str(info.contents.CameraID)
        cv2.imshow(handle, img)
        if self.num_frames_acquired == num:
            t_stop = time.monotonic()
            print("Time to capture frame:", (t_stop - t_start) / num)
            # self.c.timer.stop()
            # self.c.cap.release()
            # del c
            self.finished_signal.emit()
            cv2.destroyWindow(handle)
        return

app = QApplication(sys.argv)
engine = MightexEngine()
engine_thread = QThread()
engine.moveToThread(engine_thread)
logger = Logger()
engine.frame_read.connect(logger.finished)
logger.finished_signal.connect(engine.stop)
engine_thread.start()
num = 10000
t_start = time.monotonic()
engine.start_signal.emit()
sys.exit(app.exec_())