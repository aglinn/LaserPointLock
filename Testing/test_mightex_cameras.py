from packages.cameras import MightexEngine
from packages.cameras import MightexCamera as camera
from PyQt5.QtCore import pyqtSlot, QObject, pyqtSignal, QThread, QTimer
from PyQt5.QtWidgets import QApplication
import sys
import time
import numpy as np
import cv2

class Logger(QObject):
    finished_signal = pyqtSignal()

    def __init__(self, c, id):
        super().__init__()
        self.num_frames_acquired = 0
        self.c = c
        self.id = id
        return

    @pyqtSlot(np.ndarray)
    def finished(self, img):
        self.num_frames_acquired += 1
        handle = "camera " + str(self.id)
        cv2.imshow(handle, img)
        if self.num_frames_acquired == num:
            t_stop = time.monotonic()
            print("Time to capture frame:", (t_stop - t_start) / num)
            # self.c.timer.stop()
            # self.c.cap.release()
            # del c
            self.finished_signal.emit(self.c)
            cv2.destroyWindow(handle)
        return

num_cameras_running = 0
@pyqtSlot(object)
def stop(camera):
    global num_cameras_running, logger1, logger2, c1, c2
    timer.timeout.disconnect(camera.get_frame)
    num_cameras_running -= 1
    if num_cameras_running == 0:
        print("My cameras captured this many frames each: ", logger1.num_frames_acquired, logger2.num_frames_acquired)
        logger1.deleteLater()
        logger2.deleteLater()
        c1.deleteLater()
        c2.deleteLater()
        timer.stop()
        timer.deleteLater()
    elif num_cameras_running<0:
        print("Uh Oh!")
    return

app = QApplication(sys.argv)
timer = QTimer()
engine = MightexEngine()

c1 = camera(engine, serial_no=engine.serial_no[0])
logger1 = Logger(c1, 1)
num_cameras_running += 1
cam1_thread = QThread()
c1.moveToThread(cam1_thread)
c1.img_captured_signal.connect(logger1.finished)
timer.timeout.connect(c1.get_frame)
cam1_thread.start()

c2 = camera(engine, serial_no=engine.serial_no[1])
logger2 = Logger(c2, 2)
num_cameras_running += 1
cam2_thread = QThread()
c2.moveToThread(cam2_thread)
c2.img_captured_signal.connect(logger2.finished)
timer.timeout.connect(c2.get_frame)
cam2_thread.start()

logger1.finished_signal.connect(stop)
logger2.finished_signal.connect(stop)

_, frame = c1.cap.read()
print(frame.dtype)
print(np.max(frame))

num = 1000
fps = 30
timer.setInterval(int(1000/fps))
t_start = time.monotonic()
timer.start()
sys.exit(app.exec_())