import numpy as np
import cv2
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSlot, pyqtSignal


class BaseCamera(QObject):
    """
    This class is meant to be the parent of any camera class, it has the necessary minnimum signals and slots, but all
    slots must be overwritten with the appropriate functions for the camera being implemented. Slots are left as they
    were written for the blackfly_S to provide a bit of a guide, but they should be overwritten.


    BaseCamera was written to essentially operate the BlackflyS, but with an init function that was generic, which
    allows other cameras to inherit from BaseCamera, where other cameras may overwrite all methods of BaseCamera, but
    BlackflyS by design only needs to add a bit to the init function to operate correctly. Also, this means updates to
    BlackflyS Should be done through the BaseCamera.

    """
    # signals needed to run a camera object on its own thread.
    img_captured_signal = pyqtSignal(np.ndarray, float)
    exposure_updated_signal = pyqtSignal(float)
    gain_updated_signal = pyqtSignal(float)
    ROI_applied = pyqtSignal(bool)
    r0_updated_signal = pyqtSignal(np.ndarray)
    request_update_timer_interval_signal = pyqtSignal(float, float)
    updated_image_size = pyqtSignal(int, int)
    report_fps_read_signal = pyqtSignal(float)

    def __init__(self, dev_id: int):
        super().__init__()
        # Assume the camera is ready to acquire by the end of init function, override if neccessary.
        self._ready_to_acquire = True
        # Need to appropriately set a serial number for the camera, which is used in the camera device list of GUI to
        # connect camera.
        self.serial_no = str(dev_id)
        # Offset of the pixel coordinates, (0,0) because full view is used.
        self._startXY = [0, 0]
        # Used to toggle close function between swapping  cameras and closing the whole app
        self._app_closing = False
        # Will need to overwrite framerate with appropriate number.
        self.frame_rate = 1
        self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
        self.timeout_time = int(self.timeout_time)
        self.starting = True
        self._ROI_bounds = None
        # Unused for BlackflyS, but handle wrapping time stamps when necessary.
        self._time = 0
        self._time_offset = 0
        self.is_boson = False
        self._exposure_time = self.timeout_time
        self.camID = None
        # Variables for calculating fps
        self.t_read_start = None
        self.t_read_finish = None
        self.fps = 0
        self.num_frames_read = 0
        return

    @pyqtSlot()
    def get_frame(self):
        """
        Must capture a new frame and the time_stamp in ms. Then, emit img_captured_signal(frame, time_stamp).
        """
        ret, frame, time_stamp = self.cap.read()
        if ret:
            # Timestamp in ns as int convert to ms as float int-->float automatic.
            time_stamp = time_stamp * 10 ** -6
            self.img_captured_signal.emit(frame, time_stamp)
            self.calculate_fps(time_stamp)
        return

    def calculate_fps(self, time_stamp):
        """
        Keep track of time to calculate an fps.
        """
        t = time_stamp / 1000.0  # seconds
        self.t_read_finish = t
        if self.t_read_start is not None:
            self.fps += 1/(self.t_read_finish-self.t_read_start)
            self.num_frames_read += 1
        self.t_read_start = t

    @pyqtSlot()
    def report_fps_read(self):
        """
        Update the GUI with the average fps for frames read so GUI can display. Average interval is set by
        QTimer in the GUI thread.
        """
        if self.num_frames_read != 0:
            fps_read = self.fps/self.num_frames_read
            self.report_fps_read_signal.emit(fps_read)
        return

    @pyqtSlot(float)
    def set_exposure_time(self, t):
        """
        Set's the camera exposure time. Updates the expected time between frames (timeout_time) and emits
        request_update_timer_interval_signal with new timeout_time, which the update manager uses to update its timer
        interval. Finally, emits exposure_updated_signal with new exposure time so that GUI updates
        displayed exposure time.
        FYI, The update manager uses a timer to request a new image to be captured to make sure that the
        cameras are throttled and do not overload the update manager. So, every timeout_time ms's the update manager
        requests a new frame, or if the update manager is too busy, they request a new image when they get a chance to
        check and at least timeout_time has passed.
        t in ms
        """
        t *= 1000  # Convert from ms to us
        self.cap.set(cv2.CAP_PROP_EXPOSURE, t)
        self._exposure_time = self.cap.get(cv2.CAP_PROP_EXPOSURE) / 1000.0  # Convert back to ms
        # Update the timer interval as needed.
        frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        if self.frame_rate != frame_rate or self.starting:
            self.starting = False
            self.frame_rate = frame_rate
            self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
            self.timeout_time = int(self.timeout_time)
            self.request_update_timer_interval_signal.emit(self.timeout_time, frame_rate)
        self.exposure_updated_signal.emit(self.exposure_time)
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.updated_image_size.emit(frame_width, frame_height)
        return

    @pyqtSlot(float)
    def set_gain(self, gain):
        """
        Set the camera gain, and emit gain_updated_signal with new gain.
        """
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self._gain = self.cap.get(cv2.CAP_PROP_GAIN)
        self.gain_updated_signal.emit(self.gain)
        return

    @property
    def exposure_time(self):
        return self._exposure_time

    @property
    def gain(self):
        return self._gain

    def setup_camera_for_image_acquisition(self):
        """
        Set any parameters and/or perform any priming functions necessary to begin acquisition.
        """
        if not self._ready_to_acquire:
            pass
        return

    @pyqtSlot(bool)
    def stop_capturing(self, app_closing=False):
        """
        Called when the user is changing which camera is connected to a camview.
        """
        self._app_closing = app_closing
        # Calls close function with not closing flag, to tell camera thread to delete camera, which then triggers on
        # GUI thread the reconnect camera function.
        self.close()
        return

    @pyqtSlot()
    def close(self):
        """
        Handle closing the camera. 2 cases: 1) closing the app, where the GUI will delete the camera object after the
        camera thread is stopped, hence cannot use deleteLater. 2) just changing which camera is connected as cam1 or
        cam2, in which case, the camera thread is not closing, so use deleteLater. In both cases, close the capture
        resource, and/or release camera as instructed by camera guides.
        """
        self.cap.release()
        if not self._app_closing:
            # So far only calling like this to disconnect the camera in an effort to reconnect another. So, delete this
            # object, but do not delete the thread, which is what emitting cap_released signal does.
            self.deleteLater()
        return

    @property
    def t(self):
        """
        time that the image was acquired at in ms.
        """
        return self._time

    @t.setter
    def t(self, value):
        """
        Make sure that the timestamp of the camera is set as a monotonic function. So, you may need to be applying an
        offset if your timestamp is wrapping. For BlackflyS That wrapping time is years... so, no need and this is
        unused.
        """
        self._time = self._time_offset + value
        return

    @pyqtSlot()
    def apply_ROI(self):
        """
        Allow the user to apply an ROI to your camera, no need to read pixels that fall significantly outside of the
        laser beam profile. Setting the ROI on your camera allows faster frame rates and quicker data processing of the
        image.

        Use the self.ROI_bounds that are set graphically in the GUI by the user to then apply that ROI on your actual
        camera however that is done. E.G. For BlackflyS, the acquisition had to be stopped to apply an ROI, so do
        appropriate things as needed.
        Update the start_XY as necessary, so that the measured position is the same when there is no ROI and after you
        apply an ROI.
        Update the frame rate and the timeout time and let the UpdateManager know the new timeout time as necessary.
        FYI, The update manager uses a timer to request a new  image to be captured to make sure that the
        cameras are throttled and do not overload the update manager. So, every timeout_time ms's the update manager
        requests a new frame, or if the update manager is too busy, they request a new image when they get a chance to
        check and at least timeout_time has passed.
        """
        if self.ROI_bounds is not None:
            # Get parameters to apply to the ROI settings of Camera
            width = int(np.round(self.ROI_bounds[1] - self.ROI_bounds[0]))
            height = int(np.round(self.ROI_bounds[3] - self.ROI_bounds[2]))
            width -= (width - 8) % 4  # make this difference divisible by 4
            height -= (height - 6) % 2
            x = int(np.round(self.ROI_bounds[2]))
            y = int(np.round(self.ROI_bounds[0]))
            x -= x % 4
            y -= y % 2
            # Must make changes while not acquiring images. So, end acquisition, apply changes, begin acquisition
            self.cap.cam.EndAcquisition()
            self._ready_to_acquire = False

            x_offset = self.cap.get_pyspin_value('OffsetX')
            y_offset = self.cap.get_pyspin_value('OffsetY')
            print(x, y, width, height)
            if (x_offset == 0 and x != 0) or \
                    (y_offset == 0 and y != 0):
                # Then, I am trying to apply an offset and there is no offset yet, so it is better to apply the widths
                # first, so that I can apply the appropriate offset. If full width/height, then offset can only be (0,0)
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)
                self.cap.set_pyspin_value('OffsetX', x)
                self.cap.set_pyspin_value('OffsetY', y)
            elif x_offset == x and y_offset == y:
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)
            elif (x_offset != 0 and x == 0) or (y_offset != 0 and y == 0):
                # Then, I need to apply the offsets first, so that I am allowed to set full width/height
                self.cap.set_pyspin_value('OffsetX', x)
                self.cap.set_pyspin_value('OffsetY', y)
                self.cap.set_pyspin_value('Width', width)
                self.cap.set_pyspin_value('Height', height)

            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            # Restart Acquisition/resetup camera to acquire images:
            self.cap.cam.BeginAcquisition()
            self._ready_to_acquire = True
            self.ROI_applied.emit(True)
            frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
            if frame_rate != self.frame_rate:
                self.frame_rate = frame_rate
                self.timeout_time = np.floor((1 / self.frame_rate) * 1000)  # in ms
                self.timeout_time = int(self.timeout_time)
                self.request_update_timer_interval_signal.emit(self.timeout_time, frame_rate)
            frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.updated_image_size.emit(frame_width, frame_height)
        return

    @property
    def ROI_bounds(self):
        # xmin, xmax, ymin, ymax
        return self._ROI_bounds

    @pyqtSlot(list)
    def set_ROI_bounds(self, roi: list):
        """
        set the ROI Bounds from GUI request and then apply them.
        """
        self._ROI_bounds = roi
        self.apply_ROI()
        return

    @property
    def startXY(self):
        return self._startXY

    @startXY.setter
    def startXY(self, XY):
        """
        Update the start XY of the pixel coordinates. Then, emit r0_updated_signal so that GUI and Update Manager know
        how to interpret image data received.
        """
        self._startXY = [XY[0], XY[1]]
        self.r0_updated_signal.emit(np.asarray(self._startXY))
        return

    @pyqtSlot()
    def ensure_full_view(self):
        """
        This function should just make sure that there is no ROI applied to the camera, i.e. the camera is operating
        with the largest frame size possible.
        """
        try:
            self.cap.cam.EndAcquisition()
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            self.cap.cam.BeginAcquisition()
            self.ROI_applied.emit(False)
        except:
            self.cap.set_pyspin_value('OffsetX', 0)
            self.cap.set_pyspin_value('OffsetY', 0)
            self.cap.set_pyspin_value('Width', 100000)
            self.cap.set_pyspin_value('Height', 100000)
            self.startXY = [self.cap.get_pyspin_value('OffsetX'), self.cap.get_pyspin_value('OffsetY')]
            self.ROI_applied.emit(False)
        return