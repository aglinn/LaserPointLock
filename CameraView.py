from pyqtgraph import ImageView
from camera import Camera
import numpy as np

def calc_com(img):
    """
    Given an image, img, calculate the center of mass in pixel coordinates
    """
    Nx, Ny = img.shape
    x = np.arange(Nx)
    y = np.arange(Ny)
    X, Y = np.meshgrid(x, y, indexing='ij')
    w = img/np.sum(img)
    com_x = np.sum(X*w)
    com_y = np.sum(Y*w)
    return com_x, com_y

def add_to_data(data, plot, point, maxSize=100):
    """
    Scrolling plot with a maximum size
    """
    if (data.size < maxSize):
        data = np.append(data, point)
    else:
        data = np.roll(data, -1)
        data[-1] = point
        data = np.roll(data, -1)
        data[-1] = point
    plot.setData(data)
    return data

class CameraView():
    """
    Data structure that holds information about a camera paired to a 
    """
    def __init__(self, imageView, cam, com_x_line, com_y_line, com_x_data, com_y_data):
        self._imageView = imageView
        self._threshold = 0
        self._cam = cam
        self._img_min = 0
        self._img_max = 255
        self.saturated = False
        self.under_saturated = False
        self.com_x_line = com_x_line
        self.com_y_line = com_y_line
        self.com_x_data = com_x_data
        self.com_y_data = com_y_data
    
    @property
    def theshold(self):
        return self._threshold
    
    @threshold.setter
    def threshold(self, v):
        self._threshold = v
    
    def getFrame(self):
        img = self._cam.get_frame()
        img[img < self._threshold] = 0
        self.under_saturated = np.all(img < self._img_min)
        if not self.under_saturated:
            self.saturated = np.any(img > self._img_max)
            if not self.under_saturated:
                com_x, com_y = calc_com(img)
            else:
                com_x, com_y = -1, -1
        else:
            com_x, com_y = -1, -1
        if com_x >= 0:
            self.com_x_line.setPos(com_x)
            add_to_data(self.com_x_data, self.com_x_line, com_x)
        if com_y >= 0:
            self.com_y_line.setPos(com_y)
            add_to_data(self.com_y_data, self.com_y_line, com_y)
