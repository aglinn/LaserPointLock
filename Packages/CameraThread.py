from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np

class CameraThread(QThread):
    update_display_signal = pyqtSignal(np.ndarray)

    def __init__(self, camera_init: dict, cam_type:str):
        super().__init__()
        if 'fly' in cam_type:
            from Packages.camera import BlackflyS_EasyPySpin
            self.cam = BlackflyS_EasyPySpin(camera_init)
        self._keep_capturing = True
        self._r0 = self.cam.startXY
        self._set_pos = np.asarray([0, 0])

    def run(self):
        """
        Capture image from camera and return as nd.array to GUI to display
        """
        while self._keep_capturing:
            img = self.cam.get_frame()
            if img is not None:
                com = self.calc_com(img)
                self.update_display_signal.emit(img, com, self._r0)
        return

    def calc_com(self, img):
        """
        Given an image, img, calculate the center of mass in pixel coordinates
        """
        if not np.sum(img) == 0:
            Nx, Ny = img.shape
            x = np.arange(Nx)
            y = np.arange(Ny)
            X, Y = np.meshgrid(x, y, indexing='ij')
            # Apply offset of starting pixel coordinate for ROI purposes
            X += self._r0[0]
            Y += self._r0[1]
            w = img / np.sum(img)
            com_x = np.sum(X * w)
            com_y = np.sum(Y * w)
            return np.asarray(com_x, com_y)
        else:
            return self._set_pos