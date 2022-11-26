from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np

class CameraThread(QThread):
    update_frame_signal = pyqtSignal(np.ndarray)

    def __init__(self, camera_init: dict, cam_type:str):
        super().__init__()
        if 'fly' in cam_type:
            from Packages.camera import BlackflyS
            self.cam = BlackflyS(camera_init)
        self._keep_capturing = True

    def run(self):
        """
        Capture image from camera and return as nd.array to GUI to display
        """
        while self._keep_capturing:
            img = self.cam.get_frame()
            self.update_frame_signal.emit(img)
        return