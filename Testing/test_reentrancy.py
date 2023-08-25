import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, QObject, pyqtSlot, QMutex, QTimer, QMetaObject, Q_ARG
from PyQt5.Qt import Qt
from PyQt5.QtWidgets import QApplication
import sys
import cv2
import time


class Writer(QObject):

    send_data_signal = pyqtSignal(int, np.ndarray)
    write_data_signal = pyqtSignal()

    def __init__(self):
        super(Writer, self).__init__()
        self.data = [None, None]
        self.locks = [QMutex(), QMutex()]
        self.write_data_signal.connect(self.write, type=Qt.QueuedConnection)
        self.fps = 120
        self.recording = True
        self.write_id = 0
        return

    @pyqtSlot()
    def write(self):
        if self.recording:
            self.write_id += 1
            self.write_id = self.write_id % 2
            dat = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
            self.locks[self.write_id].lock()
            # print("acquired lock, writing  data with id {}".format(self.write_id))
            self.data[self.write_id] = dat
            self.locks[self.write_id].unlock()
            # print("released lock, after writing  data with id {}".format(self.write_id))
            self.write_data_signal.emit()
            # time.sleep(1/(2*self.fps))
        else:
            return
        return

    def send_data(self, id):
        # print("Called read on camera id {}.".format(id))
        self.locks[id].lock()
        # print("acquired lock, reading  data with id {}".format(id))
        dat = self.data[id]
        if dat is None:
            self.locks[id].unlock()
            # print("released lock, after reading  data with id {}".format(id))
            return
        self.send_data_signal.emit(id, dat)
        self.data[id] = None
        self.locks[id].unlock()
        # print("released lock, after reading  data with id {}".format(id))
        return


class Sender(QObject):
    report_data_signal = pyqtSignal()

    def __init__(self, send_data, id: int):
        super(Sender, self).__init__()
        self.send_data = send_data
        self.id = id
        self.fps = 120
        return

    @pyqtSlot()
    def report_data(self):
        self.send_data(self.id)
        # time.sleep(1/self.fps)
        return


class Logger(QObject):
    finished_signal = pyqtSignal()

    def __init__(self, request_frame_signal):
        super().__init__()
        self.num_frames_acquired = [0, 0]
        self.running_cams = 2
        self.request_frame_signal = request_frame_signal
        self.timers = [QTimer(self), QTimer(self)]
        self.timers[0].setSingleShot(True)
        self.timers[1].setSingleShot(True)
        self.timers[0].setInterval(int(1000/120.0))
        self.timers[1].setInterval(int(1000/120.0))
        self.timers[0].timeout.connect(lambda:
                                       QMetaObject.invokeMethod(self, 'display', Qt.QueuedConnection, Q_ARG(int, 0),
                                                                Q_ARG(np.ndarray, np.empty(1))))
        self.timers[1].timeout.connect(lambda:
                                       QMetaObject.invokeMethod(self, 'display', Qt.QueuedConnection, Q_ARG(int, 1),
                                                                Q_ARG(np.ndarray, np.empty(1))))
        return

    @pyqtSlot(int, np.ndarray)
    def display(self, id, img):
        if self.timers[id].isActive():
            self.timers[id].stop()
            self.num_frames_acquired[id] += 1
        else:
            self.request_frame_signal[id].emit()
            self.timers[id].start()
            return
        handle = "camera " + str(id)
        # print("Displaying img from camera with id {}.".format(id), img.shape, handle)
        cv2.imshow(handle, img)
        if self.num_frames_acquired[id] == num:
            self.running_cams -= 1
            cv2.destroyWindow(handle)
            if self.running_cams == 0:
                t_stop = time.monotonic()
                print("Time to capture frame:", (t_stop - t_start) / (2*num))
                # self.c.timer.stop()
                # self.c.cap.release()
                # del c
                self.finished_signal.emit()
        else:
            self.request_frame_signal[id].emit()
            self.timers[id].start()
        return

@pyqtSlot()
def stop():
    writer.deleteLater()
    sender1.deleteLater()
    sender2.deleteLater()
    logger.deleteLater()
    time.sleep(2)
    """writer_thread.quit()
    writer_thread.wait()
    del writer_thread
    sender1_thread.quit()
    sender1_thread.wait()
    del sender1_thread
    sender2_thread.quit()
    sender2_thread.wait()
    del sender2_thread
    logger_thread.quit()
    logger_thread.wait()
    del logger_thread"""
    app.quit()
    return

app = QApplication(sys.argv)
# Setup writer
writer = Writer()
writer_thread = QThread()
writer.moveToThread(writer_thread)
writer_thread.start()

#Setup 2x senders
sender1 = Sender(writer.send_data, 0)
sender1_thread = QThread()
sender1.moveToThread(sender1_thread)
sender1_thread.start()
sender1.report_data_signal.connect(sender1.report_data)
sender2 = Sender(writer.send_data, 1)
sender2_thread = QThread()
sender2.moveToThread(sender2_thread)
sender2_thread.start()
sender2.report_data_signal.connect(sender2.report_data)

#Setup Logger
"""logger = Logger([sender1.report_data_signal, sender2.report_data_signal])
logger_thread = QThread()
logger.moveToThread(logger_thread)
logger_thread.start()"""
logger = Logger([sender1.report_data_signal, sender2.report_data_signal])
writer.send_data_signal.connect(logger.display)
logger.finished_signal.connect(stop)

num = 10000
t_start = time.monotonic()
print(logger.thread(), logger.timers[0].thread(), logger.timers[1].thread())
writer.send_data_signal.emit(0, np.empty(1))
writer.send_data_signal.emit(1, np.empty(1))
writer.write_data_signal.emit()
sys.exit(app.exec_())