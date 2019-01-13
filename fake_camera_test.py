from camera import FakeCamera
from pyqtgraph import ImageView
from PyQt5 import QtGui, QtCore

c = FakeCamera()

app = QtGui.QApplication([])
win = QtGui.QMainWindow()
win.resize(800,800)
imv = ImageView()
win.setCentralWidget(imv)
win.show()
win.setWindowTitle('pyqtgraph example: ImageView')

c.set_resolution(1280, 1024)
img = c.get_frame()
print(img)
print(img.min(), img.max())
imv.setImage(img)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
