import numpy as np
import cv2
import time
from flirpy.camera.boson import Boson
from flirpy.camera.core import Core
from PyQt5.QtCore import pyqtSlot
from packages.cameras.base import BaseCamera


class BosonQObject(Boson, BaseCamera):
    # TODO: Can I set an ROI or not? No documentation that indicates that I can, but documentation says it is a fully
    # compliant USB video device. So, maybe that implies I can set an ROI using cv2?

    def __init__(self, port=None, device_id=None):
        # MRO: self, Boson, Core, BaseCamera, QObject, sip.wrapper, sip.simplewrapper, object
        # Init Boson class first.
        self.port = port
        super().__init__(port=self.port)
        serial_no = str(self.get_camera_serial())
        # With serial number in hand, can init BaseCamera Object
        super(Core, self).__init__(dev_id=serial_no)
        ##############################################################
        # Additional steps to finish initing the BosonQObject class #
        ##############################################################
        self.frame = None
        # self.set_ffc_manual()
        self.set_ffc_auto()
        print("FFC Mode:", self.get_ffc_mode())
        if not self.get_gao_ffc_mode():
            print("The ffc correction is not being applied.")
        if not self.get_ffc_mode() == 0:
            print("The IR cameras need to be run in manual FFC mode.")
        # Find device by device_id
        self.device_id = self.find_video_device(device_id=device_id)
        # Setup the video device as a cv2 capture device.
        self.setup_video(self.device_id)
        self.set_ROI_bounds([0, 1024, 0, 1280])
        self.is_boson = True
        # Will need to overwrite framerate with appropriate number.
        # Cannot read Boson frame rate
        # self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        self.timeout_time = np.floor((1 / 60.0) * 1000)  # in ms
        self.timeout_time = int(self.timeout_time)
        self._exposure_time = self.timeout_time  # uniqute to Boson that does not have a settable exposure time.
        # TODO: Make sure that timer interval is updated with correct signal.
        # Unused for BlackflyS, but handle wrapping time stamps when necessary.
        self._time_offset = time.monotonic()*1000 # in ms.
        self._time = self._time_offset
        return


    def get_frame(self):
        """
        Must capture a new frame and the time_stamp in ms. Then, emit img_captured_signal(frame, time_stamp).
        Boson does not provide the timestamp of the image, sadly. So, I must use a manual timestamp, hence the use of
        time.monotonic().
        """
        self.update_frame()
        self.t = time.monotonic()*1000  # in ms
        # Boson does not allow me to apply an ROI to the camera such that only those pixels are read. Thus, I apply an
        # ROI only manually as a means of croping the data after it is read. This does not get a speedup in FPS, but it
        # does still improve processing time of the images and importantly allows clipping unwanted data (bright pixels
        # by the edge of the detector.
        self.img_captured_signal.emit(self.frame[int(self.ROI_bounds[0]):int(self.ROI_bounds[1] + 1),
               int(self.ROI_bounds[2]):int(self.ROI_bounds[3] + 1)], self.t)
        return

    def update_frame(self):
        """
        Grab a frame from the camera and store it as a uint16.
        """
        self.frame = self.grab(device_id=self.device_id)
        return

    @property
    def t(self):
        return self._time

    @t.setter
    def t(self, value):
        """
        had to use a software timestamp :( So, using time.monotonic and want to subtract the initial time at initing the
        camera object.
        """
        self._time = value - self._time_offset
        return

    @pyqtSlot(list)
    def set_ROI_bounds(self, roi: list):
        """
        set the ROI Bounds from GUI request and then apply them.
        """
        self._ROI_bounds = np.round(roi)
        self.apply_ROI()
        return

    def apply_ROI(self):
        """
        Unfortunately, I cannot apply an ROI on the camera hardware to avoid waisting time reading unwanted pixels. So,
        this function will be different than for BlackflyS.

        Just make sure that the startXY or image origin in pixel coordinates is updated and that other objects know what
        startXY is..
        """
        if self.ROI_bounds is not None:
            # Only need to update the startXY of the image that camera will be emitting.
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))

            self.startXY = [x, y]
            self.ROI_applied.emit(True)
        return

    def service(self):
        pass

    def ensure_full_view(self):
        """
        I am unable to apply an ROI on the actual camera, so camera will always return full view. i.e. never apply an
        ROI in the first place.
        """
        pass

    @pyqtSlot()
    def close(self):
        """
        Handle closing the camera. 2 cases: 1) closing the app, where the GUI will delete the camera object after the
        camera thread is stopped, hence cannot use deleteLater. 2) just changing which camera is connected as cam1 or
        cam2, in which case, the camera thread is not closing, so use deleteLater. In both cases, close the capture
        resource, and/or release camera as instructed by camera guides.
        """
        self.release()
        if not self._app_closing:
            # So far only calling like this to disconnect the camera in an effort to reconnect another. So, delete this
            # object, but do not delete the thread, which is what emitting cap_released signal does.
            self.deleteLater()
        return

    @pyqtSlot(float)
    def set_exposure_time(self, t):
        """
        Boson's do not have an exposure time. But, I am going to call this function on initializing the camera to
        tell the updatemanager to start the timer for grabbing frames, and also to tell the Update Manager what the
        "exposure time" is, since this is used in I term of PID. "exposure time" will just be considered 1/fps in ms or
        the same as timeout_time.
        """
        # both exposure time and timeout time are set in the init function and never change.
        self.request_update_timer_interval_signal.emit(self.timeout_time)
        self.exposure_updated_signal.emit(self.exposure_time)
        # TODO: Can I read exposure? Thus could I set an exposure time?
        print("Reading the Boson exposure time as ", self.cap.get(cv2.CAP_PROP_EXPOSURE))
        return

    @pyqtSlot(float)
    def set_gain(self, gain):
        """
        Set the camera gain, and emit gain_updated_signal with new gain.
        """
        # TODO: Can I use Gain like this or not with the BOSON?
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self._gain = self.cap.get(cv2.CAP_PROP_GAIN)
        self.gain_updated_signal.emit(self.gain)
        return