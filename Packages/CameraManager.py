import numpy as np
from camera import MightexCamera
from PyQt5.QtCore import QObject
from Packages.camera import MightexCamera

class Cam_Handler(QObject):

    def __init__(self, cam, cam_index):
        self.cam = cam
        self.cam_index = cam_index
        self.cam.update_frame()

        # init parameters:
        self.x_line = None
        self.y_line = None
        self.cam_lock_symbol = None
        self.cam_unlock_symbol = None
        self.cam_x_plot = None
        self.cam_y_plot = None
        self.le_cam_max = None
        self.gv_camera = None
        self._suppress_image_display = False
        self._com = []
        self._t = []
        self.max_vector_length = 100
        self.number_frames_average = 1
        self.set_pos = [0,0]
        self.resetView = True
        self._threshold = 0
        self.get_threshold()

    def acquire_and_display(self):
        """
        Acquire a new image and display, process image, update GUI with img and pointing info
        Given the camera at cam_index in the camera list
        Update cam_view with its image and COM
        Use threshold to zero out values below threshold
        Return if the image is saturated
        """

        try:
            # Should only work if the camera has a service function, which boson will others won't
            self.cam.service()
        except:
            pass

        # Get an image that is an average over the number of AvgFrames
        if self.number_frames_average < 2:
            try:
                self.cam.update_frame()
                img = self.cam.get_frame()
            except RuntimeError:
                self.cam_lock_symbol.setVisible(False)
                self.cam_unlock_symbol.setVisible(True)
                img = np.array([1])
        else:
            for i in range(self.number_frames_average):
                try:
                    self.cam.update_frame()
                    temp_img = self.cam.get_frame()
                except RuntimeError:
                    self.cam_lock_symbol.setVisible(False)
                    self.cam_unlock_symbol.setVisible(True)
                    temp_img = np.array([1])
                    img = np.array([1])
                    break
                if i == 0:
                    img = temp_img
                else:
                    img += temp_img
            img = img / self.number_frames_average

        #Grab time from frame:
        self.t = self.cam.time

        # Apply threshold
        if self.threshold > 0:
            if np.any(img > self.threshold):
                img[img < self.threshold * self.number_frames_average] = 0

        # Get center of mass on the image
        r_0 = self.cam.startXY
        self.com = self.calc_com(img, set_position=self.set_pos, r_0=r_0)

        ##############
        # Update GUI #
        ##############

        # Update max value of camera image:
        self.le_cam_max.setText(str(np.max(img / self.number_frames_average)))

        # Update COM crosshairs on the image:
        self.x_line.setPos(self.com[-1,0])
        self.y_line.setPos(self.com[-1,1])
        self.x_line.setVisible(True)
        self.y_line.setVisible(True)

        # Update the pointing position plots:
        if not self.suppress_pointing_display:
            self.cam_x_plot.setData(self.com[:,0])
            self.cam_y_plot.setData(self.com[:,1])

        # Update the image:
        if not self.suppress_image_display:
            if self.resetView:
                self.gv_camera.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False, pos=r_0)
                self.resetView = False
            else:
                self.gv_camera.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False, pos=r_0)
        return

    @staticmethod
    def calc_com(img, set_position,  r_0=[0, 0]):
        """
        Given an image, img, calculate the center of mass in pixel coordinates
        """
        Nx, Ny = img.shape
        x = np.arange(Nx)
        y = np.arange(Ny)
        X, Y = np.meshgrid(x, y, indexing='ij')
        # Apply offset of starting pixel coordinate for ROI purposes
        X += r_0[0]
        Y += r_0[1]
        try:
            w = img/np.sum(img)
        except RuntimeWarning:
            # If dividing by 0, then the frame is empty. So return set position as COM.
            return set_position
        com_x = np.sum(X*w)
        com_y = np.sum(Y*w)
        return com_x, com_y

    def report_to_update_manager(self):
        #TODO: Make sure that I am reporting the correct COM to update manager, factoring in pixel starting coordinates
        return self.t, self.com[-1]

    def report_settings(self):
        #TODO: See update_cam1_settings()
        pass

    def update_GUI(self, max_value: str):

    def update_settings(self):
        #TODO: See update_cam1_settings()
        pass
        return

    def update_display_params(self, x_line, y_line, cam_lock_symbol, cam_unlock_symbol, cam_x_plot, cam_y_plot,
                              le_cam_max, gv_camera):
        self.x_line = x_line
        self.y_line = y_line
        self.cam_lock_symbol = cam_lock_symbol
        self.cam_unlock_symbol = cam_unlock_symbol
        self.cam_x_plot = cam_x_plot
        self.cam_y_plot = cam_y_plot
        self.le_cam_max = le_cam_max
        self.gv_camera = gv_camera
        return

    @property
    def com(self):
        return np.asarray(self._com)

    @com.setter
    def com(self, vector):
        self._com.append(vector)
        if len(self._com) > self.max_vector_length:
            del self._com[0]
        return

    @property
    def t(self):
        return self._t

    @t.setter
    def t(self, time: float):
        self._t.append(time)
        if len(self._t)>self.max_vector_length:
            del self._t[0]
        return

    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, threshold: float):
        self._threshold = threshold
        self.resetView = True
        return

    def get_threshold(self):
        """
        Want to make this send signal to GUI to get the value set in the GUI. Not implemented.
        """
        self.threshold = 0
        return

    @property
    def suppress_image_display(self):
        return self._suppress_image_display

    @suppress_image_display.setter
    def suppress_image_display(self, truth: bool):
        self._suppress_image_display = truth
        return

    @property
    def suppress_pointing_display(self):
        return self._suppress_image_display

    @suppress_pointing_display.setter
    def suppress_pointing_display(self, truth: bool):
        self._suppress_pointing_display = truth
        return

    def close(self):
        pass