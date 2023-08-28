import numpy as np
from packages.cameras.base import BaseCamera
from packages.cameras.Mightex.mightex_engine import MightexEngineSignals
from PyQt5.QtCore import pyqtSlot


class MightexCamera(BaseCamera):

    def __init__(self, send_frame, get_id, engine_signals: MightexEngineSignals, serial_no: str):
        """
        This class primarily exists to allow backwards compatibility with the application as written to hold and
        communicate with two camera objects. That worked great for FLIR cameras; however, the Mightex camera
        communication occurs through an engine layer, and just to be safe for multithreading purposes, the
        MightexEngine will handle all communication with the cameras and all grabbing, so this object primarily serves
        as an intermediary between the application and the engine. Thus, this object mostly passes signals. Only the
        actual frame grabbing is updated slightly to avoid an additional copy by passing the signal for speed purposes.
        __inputs__
        send_frame: function, MightexEngine.send_frame
        get_id: function, MightexEngine.get_id
        engine_signals: instance of MightexEngineSignals that MightexEngine Instantiates
        serial_no: string, serial number of the camera.
        """
        super().__init__(serial_no)
        self.serial_no = serial_no
        self._ROI_bounds = None
        self.send_frame = send_frame
        self.camID = get_id(serial_no)
        self.engine_signals = engine_signals
        self.connect_signals()
        return

    def connect_signals(self):
        """
        The Engine will emit some signals that need to be re-emitted as camera signals for backwards compatibility.
        Only the img_updated signal will be directly connects without this object as an intermediary for speed.
        Also, the camera signals out to the engine need to be connected by the engine. Send signal to tell engine to do
        so.
        """
        # From Engine
        self.engine_signals.updated_exposure_time.connect(self.pass_engine_updated_exposure_time)
        self.engine_signals.updated_xy.connect(self.pass_engine_update_xy)
        self.engine_signals.request_update_timer_interval.connect(self.pass_engine_request_update_timer_interval)
        return

    @pyqtSlot(str)
    def activate(self):
        """
        activate the camera and tell the engine to start frame grabbing if it has not already done so!
        """
        self.engine_signals.activate_camera.emit(self.serial_no, 1)
        self.engine_signals.start_grabbing.emit()
        return

    @pyqtSlot(int, float)
    def pass_engine_updated_exposure_time(self, camID, t):
        if camID == self.camID:
            self.exposure_updated_signal.emit(t)
        return

    @pyqtSlot(int, np.ndarray)
    def pass_engine_update_xy(self, camID, xy):
        if camID == self.camID:
            self.r0_updated_signal.emit(xy)
        return

    @pyqtSlot(int, float, float)
    def pass_engine_request_update_timer_interval(self, camID, timeout_time, fps):
        if camID == self.camID:
            self.request_update_timer_interval_signal.emit(timeout_time, fps)
        return

    @pyqtSlot(int, bool)
    def pass_engine_ROI_applied(self, camID, roi_is_set):
        if camID == self.camID:
            self.ROI_applied.emit(roi_is_set)
        return

    @pyqtSlot(int, int, int)
    def pass_engine_updated_frame_size(self, camID, width, height):
        if camID == self.camID:
            self.updated_image_size.emit(width, height)
        return

    @pyqtSlot()
    def get_frame(self):
        """
        Send a frame and time stamp for this camera. I use the reentrancy of QObjects to simply
        call the send_frame function of the MightexEngine and I use locks to avoid race conditions.
        """
        self.send_frame(self.serial_no)
        return

    @pyqtSlot(float)
    def set_exposure_time(self, t):
        """
        Set's the camera exposure time.
        t in ms
        """
        self.engine_signals.set_exposure.emit(self.serial_no, t)
        if self.starting:
            self.activate()
            self.starting = False
        return

    @pyqtSlot(float)
    def set_gain(self, gain):
        """
        Set the camera gain, and emit gain_updated_signal with new gain.
        """
        if gain < 0.125:
            gain = 0.125
        elif gain > 8:
            gain = 8
        self.engine_signals.set_gain.emit(self.serial_no, gain)
        self.gain_updated_signal.emit(gain)
        return

    @property
    def startXY(self):
        return self._xy

    @property
    def exposure_time(self):
        return self._exposure_time

    def close(self):
        """
        Handle closing the camera. 2 cases: 1) closing the app, where the GUI will delete the camera object after the
        camera thread is stopped, hence cannot use deleteLater. 2) just changing which camera is connected as cam1 or
        cam2, in which case, the camera thread is not closing, so use deleteLater. In both cases, close the capture
        resource, and/or release camera as instructed by camera guides.
        """
        if not self._app_closing:
            # So far only calling like this to disconnect the camera in an effort to reconnect another. So, delete this
            # object, but do not delete the thread, which is what emitting cap_released signal does.
            self.engine_signals.activate_camera.emit(self.serial_no, 0)  # deactivate camera.
            self.engine_signals.stop_grabbing.emit()
            self.deleteLater()
        return

    @pyqtSlot()
    def apply_ROI(self):
        """
        Allow the user to apply an ROI to your camera, no need to read pixels that fall significantly outside of the
        laser beam profile. Setting the ROI on your camera allows faster frame rates and quicker data processing of the
        image.

        Use the self.ROI_bounds that are set graphically in the GUI by the user to then apply that ROI on your actual
        camera however that is done.
        """
        if self.ROI_bounds is not None:
            # The camera engine will apply the roi, and as the startXY, size, and fps change, the engine will send
            # signals accordingly. That pass back through this camera object and on to whomever.
            self.engine_signals.apply_roi.emit(self.serial_no, self.ROI_bounds)
        return

    @pyqtSlot()
    def ensure_full_view(self):
        """
        This function should just make sure that there is no ROI applied to the camera, i.e. the camera is operating
        with the largest frame size possible.
        """
        self.set_ROI_bounds([0, 1024, 0, 1280])
        return