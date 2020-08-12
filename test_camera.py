import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets
import sys

from camera import FakeCamera, MightexEngine, MightexCamera

if __name__ == '__main__':
    # Create the GUI window
    app = QtWidgets.QApplication([])
    win = QtWidgets.QMainWindow()
    win.setGeometry(100, 100, 400, 400)
    win.setWindowTitle("Test")
    canvas = pg.ImageView()
    win.setCentralWidget(canvas)

    # Create the camera
    mightex_engine = MightexEngine()
    cam = MightexCamera(mightex_engine, mightex_engine.serial_no[1])
    # Set initial data and levels on image
    canvas.setImage(np.zeros((1024, 1280)))
    canvas.setLevels(0, 255)

    # Create a threadpool object which QRunnable objects can be concurrently run with
    threadpool = QtCore.QThreadPool()
    print("Multithreading with maximum %d threads" % threadpool.maxThreadCount())
    threadpool.start(cam.run_thread)

    # Create a slot, or callback function, so that when the frame_update signal is emitted, the image changes
    def update_img():
        canvas.setImage(cam.get_frame(), autoRange=False, autoLevels=False, autoHistogramRange=False)
    # Connect the frame_update signal to the update_img slot
    cam.signals.frame_update.connect(update_img)

    # Show the window
    win.show()
    # Start the Qt event loop
    sys.exit(app.exec_())
