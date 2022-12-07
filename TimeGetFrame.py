from Packages.camera import BlackflyS_EasyPySpin_QObject as camera
from PyQt5.QtCore import pyqtSlot, QThread, QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication
import sys
import timeit
import time
import numpy as np

class Logger(QObject):
    finished_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.num_frames_acquired = 0
        return

    @pyqtSlot(np.ndarray)
    def finished(self, img):
        self.num_frames_acquired += 1
        if self.num_frames_acquired == num:
            t_stop = time.monotonic()
            print("Time to capture frame:", (t_stop - t_start) / num)
            c.timer.stop()
            c.cap.release()
            del c
            self.deleteLater()
        return

app = QApplication(sys.argv)
logger = Logger()
c = camera(1)
_, frame = c.cap.read()
print(frame.dtype)
print(np.max(frame))
c.img_captured_signal.connect(logger.finished)
num = 1000
num_frames_acquired = 0
t_start = time.monotonic()
c.timer.start()
sys.exit(app.exec_())