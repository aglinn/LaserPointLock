import numpy as np
import cv2
import EasyPySpin
from packages.cameras.base import BaseCamera


class BlackflyS(BaseCamera):
    """
    BaseCamera was written to essentially operate the BlackflyS, but with an init function that was generic, which
    allows other cameras to inherit from BaseCamera, where other cameras may overwrite all methods of BaseCamera, but
    BlackflyS by design only needs to add a bit to the init function to operate correctly. Also, this means updates to
    BlackflyS Should be done through the BaseCamera.
    """

    def __init__(self, dev_id: int):
        """
        Construct the BlackflyS camera object.
        """
        # Init the base camera. which inits QObject.
        super().__init__(dev_id)

        ########################################
        # Additional Init steps for BlackflyS #
        ########################################
        # Open Capture object
        self.cap = EasyPySpin.VideoCapture(dev_id)
        if not self.cap.isOpened():
            print("Camera can't open\nexit")
            return -1
        # Set camera settings.
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -1)  # -1 sets exposure_times to auto
        self.cap.set(cv2.CAP_PROP_GAIN, -1)  # -1 sets gain to auto
        self.cap.set_pyspin_value("GammaEnable", False)
        print("Gamma is ", self.cap.get(cv2.CAP_PROP_GAMMA))
        self._exposure_time = 'Auto'
        self.ROI_full_bounds = [0, 100000, 0, 100000]  # This is static and only ever called by the GUI thread.
        self._gain = 1
        # Get the current frame rate and use that to set the timeout time for UpdateManager's timer for grabbing the
        # next image. Must overwrite appropriately.
        self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        self.timeout_time = np.floor((1 / self.frame_rate)*1000) # in ms
        self.timeout_time = int(self.timeout_time)
        self.starting = True
        self._ROI_bounds = None
        # Setup camera to be full view rather than in ROI mode.
        self.ensure_full_view()
        return