
import time
from flirpy.camera.boson import Boson

from Packages.Cameras.Camera import Camera

class BosonCamera(Boson, Camera):

    kind = "BosonCamera"

    def __init__(self, port=None, device_id=None):
        self.port = port
        super().__init__(port=self.port)
        self.frame = None
        self.serial_no = str(self.get_camera_serial())
        # self.set_ffc_manual()
        self.set_ffc_auto()
        print("FFC Mode:", self.get_ffc_mode())
        if not self.get_gao_ffc_mode():
            print("The ffc correction is not being applied.")
        if not self.get_ffc_mode() == 0:
            print("The IR cameras need to be run in manual FFC mode.")
        self.device_id = self.find_video_device(device_id=device_id)

    def get_frame(self):
        return self.frame

    def update_frame(self):
        """
        Grab a frame from the camera and store it as a uint16.
        """
        self.frame = self.grab(device_id=self.device_id)
