# TODO: 2 stop using global vars and put this code into a fucking window
# TODO: 1 Add statistics logging.
# TODO: 1 Let me FFT the camera COM coordinates to find out which frequencies are generating the noise.
# TODO: 1 For the above two, it may be useful to have a new state or even a seperate program to run diagnostics in a super
#  lean fashion; so we can be sensitive to the highest possible frequencies.
# TODO: 1 Finish overhauling code to allow for operation with IR Cameras.
# TODO: 3 Try using OpenCV with the Mightex cameras for faster operation
# TODO: 2 Try to shutter the TOPAS if the BOSON FPAs get too hot. Can I even control the TOPAS shutter?
# TODO: 2 Stupid bug fix: When I reduce the number of points to be plotted for COM and piezzo values, I need to make the
#  number of points plotted actually smaller.
# TODO: 2 try the other algorithm from the paper I found.
# TODO: 2 Implement PID, instead of just D.
# TODO: 2 Make the GUI compatible with multiple screen formats.
# TODO: 3 multithread the program.
# TODO: 2 Generally improve the coding.
# TODO: 3 Make it possible to set a ROI on the camera; so that the cameras only read the pixels in the ROI. This will
#  likely give us quite a bit of speed up on the program update time, and it is way simpler than multithreading.
# TODO: 3 Improve communication with the piezo motors.
# TODO: 2 Log the information on Grafana.
# TODO: 3 Move all print statements to the GUI.
# TODO: 2 Could we cash the current program settings to load automatically on next open?
# TODO: 2 Add GUI ability to set the averager state of the BOSON camera, and power on defaults.
# TODO: 3 It would be nice to allow the user to switch between IR and visibile system, which would require disconnecting
#  devices and reinitializing the cam_list.
# TODO: 1 I need to be able to run multiple instances of the program; so that I can run an IR and a vis instance in
#  parallel.
# TODO: I need to make sure that the older piezzo controllers still work with this code!
from PyQt5.QtWidgets import QMainWindow
from Packages.pointing_ui import Ui_MainWindow

# Old imports:
import sys
import visa
import time
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThreadPool, QThread
from Packages.camera import MightexCamera, MightexEngine, DeviceNotFoundError, BosonCamera
from Packages.camera import BlackflyS_EasyPySpin_QObject as BlackflyS
from Packages.motors import MDT693A_Motor
from Packages.motors import MDT693B_Motor
from Packages.CameraThread import CameraThread
import tkinter as tk
from tkinter import filedialog
from serial.tools import list_ports
# from Packages.UpdateManager import UpdateManager
from Packages.UpdateManager import PIDUpdateManager as UpdateManager
from Packages.UpdateManager import InsufficientInformation, update_out_of_bounds
import copy
import pickle as pkl
import gc
import matplotlib.pyplot as plt
from Thorlabs_MDT69XB_PythonSDK import MDT_COMMAND_LIB as mdt
import PySpin

pg.setConfigOptions(imageAxisOrder='row-major')
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

"""
# Add fake cameras
for i in range(3):
   c = FakeCamera()
   cam_list.append(c)
   cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
"""

class Window(QMainWindow, Ui_MainWindow):
    STATE_MEASURE = 0
    STATE_CALIBRATE = 1
    STATE_LOCKED = 2
    STATE_ALIGN = 3
    UPDATE_TIME = 500  # ms

    ## Set a custom color map
    colors = [
        (68, 1, 84),
        (72, 33, 115),
        (67, 62, 133),
        (56, 88, 140),
        (45, 112, 142),
        (37, 133, 142),
        (30, 155, 138),
        (42, 176, 127),
        (82, 197, 105),
        (134, 213, 73),
        (194, 223, 35),
        (253, 231, 37)
    ]
    cmap = pg.ColorMap(pos=np.linspace(0.0, 1.0, 12), color=colors)

    def __init__(self):
        super().__init__()
        #Call the setupUI function
        self.setupUi(self)
        ######################################
        # Initialize all instance attirbutes #
        ######################################
        self.state = self.STATE_MEASURE
        #  Instantiate Update Manager
        self.UpdateManager = UpdateManager()

        # params for Handle threading of cameras:
        self.cam1_thread = QThread()
        self.cam2_thread = None
        self.updatemanager_thread = None
        #self.cam2_thread = QThread()
        #self.updatemanager_thread = QThread()
        self.cam1 = None
        self.cam2 = None
        # Grab the app's threadpool object to manage threads
        self.threadpool = QThreadPool.globalInstance()
        # assign item models
        self.cam_model = QtGui.QStandardItemModel()
        self.motor_model = QtGui.QStandardItemModel()
        # error dialog
        self.error_dialog = QtWidgets.QErrorMessage()
        # Camera Parameters
        self.cam_init_dict = {}
        self.cam1_threshold = 0
        self.cam2_threshold = 0
        # Motors parameters
        self.motor1_index = None
        self.motor2_index = None
        self.ResourceManager = visa.ResourceManager()
        # Holds PID Dict
        self.PID = {}
        # Initialize global variables
        # cam1_index = -1
        # cam2_index = -1
        self.cam1_threshold = 0
        self.cam2_threshold = 0
        self.cam1_reset = True
        self.cam2_reset = True
        self.most_recent_error = ''

        # Initialize global variables for Locking
        self.LockTimeStart = 0
        self.Unlocked_report = {}
        self.Unlock_counter = 0
        self.user_selected_unlock_report = False
        self.saturated_cam1 = False
        self.under_saturated_cam1 = False
        self.saturated_cam2 = False
        self.under_saturated_cam2 = False
        self.motor_list = []
        self.TimeLastUnlock = 0
        self.num_out_of_voltage_range = 1
        self.first_unlock = True  # First time since initial lock that piezos went out of bounds?
        self.set_cam1_x = None
        self.set_cam1_y = None
        self.set_cam2_x = None
        self.set_cam2_y = None

        # Initialize Global variables for home position markers:
        self.ROICam1_Unlock = None
        self.ROICam2_Unlock = None
        self.ROICam1_Lock = None
        self.ROICam2_Lock = None

        # Initialize Global Variables for arrows for Alignment Mode:
        self.Cam1_LeftArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam1_RightArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam1_DownArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam1_UpArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam2_LeftArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam2_RightArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam2_DownArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.Cam2_UpArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")

        # Initialize global variables for tracking pointing
        self.cam1_r0 = [0, 0]
        self.cam1_x = np.zeros(1)
        self.cam1_y = np.zeros(1)
        self.cam2_x = np.zeros(1)
        self.cam2_y = np.zeros(1)
        self.cam1_x_time = np.zeros(1)
        self.cam1_y_time = np.zeros(1)
        self.cam2_x_time = np.zeros(1)
        self.cam2_y_time = np.zeros(1)
        self.cam1_x_PlotItem = self.gv_cam_xy.addPlot(row=0, col=0, labels={'left': 'Cam 1 X'})
        self.cam1_y_PlotItem = self.gv_cam_xy.addPlot(row=1, col=0, labels={'left': 'Cam 1 Y'})
        self.cam2_x_PlotItem = self.gv_cam_xy.addPlot(row=2, col=0, labels={'left': 'Cam 2 X'})
        self.cam2_y_PlotItem = self.gv_cam_xy.addPlot(row=3, col=0, labels={'left': 'Cam 2 Y'})
        self.cam1_x_plot = self.cam1_x_PlotItem.plot()
        self.cam1_y_plot = self.cam1_y_PlotItem.plot()
        self.cam2_x_plot = self.cam2_x_PlotItem.plot()
        self.cam2_y_plot = self.cam2_y_PlotItem.plot()
        # cam1_x_plot_target =self.gv_cam_xy.addPlot(row=0, col=0, labels={'left': 'Cam 1 X'}).plot()
        # cam1_y_plot_target =self.gv_cam_xy.addPlot(row=1, col=0, labels={'left': 'Cam 1 Y'}).plot()
        # cam2_x_plot_target =self.gv_cam_xy.addPlot(row=2, col=0, labels={'left': 'Cam 2 X'}).plot()
        # cam2_y_plot_target =self.gv_cam_xy.addPlot(row=3, col=0, labels={'left': 'Cam 2 Y'}).plot()
        # Initialize global variables for piezo motor voltages
        self.motor1_x = np.zeros(1)
        self.motor1_y = np.zeros(1)
        self.motor2_x = np.zeros(1)
        self.motor2_y = np.zeros(1)
        self.motor1_x_plot = self.gv_piezo.addPlot(row=0, col=0).plot()
        self.motor1_y_plot = self.gv_piezo.addPlot(row=1, col=0).plot()
        self.motor2_x_plot = self.gv_piezo.addPlot(row=2, col=0).plot()
        self.motor2_y_plot = self.gv_piezo.addPlot(row=3, col=0).plot()
        self.suppress_pointing_display = False
        self.suppress_image_display = False

        # Set the com guide lines
        self.cam1_x_line = pg.InfiniteLine(movable=False, angle=0)
        self.cam1_y_line = pg.InfiniteLine(movable=False, angle=90)
        self.cam2_x_line = pg.InfiniteLine(movable=False, angle=0)
        self.cam2_y_line = pg.InfiniteLine(movable=False, angle=90)

        # Initialize global variables for calibration
        self.calib_index = 0
        self.reset_piezo = 0
        self.override = False
        self.calibration_voltages = 5 * np.arange(31)
        self.mot1_x_voltage, self.mot1_y_voltage, self.mot2_x_voltage, self.mot2_y_voltage = np.empty((4, len(self.calibration_voltages)))
        self.mot1_x_cam1_x, self.mot1_x_cam1_y, self.mot1_x_cam2_x, self.mot1_x_cam2_y = np.empty((4, len(self.calibration_voltages)))
        self.mot1_y_cam1_x, self.mot1_y_cam1_y, self.mot1_y_cam2_x, self.mot1_y_cam2_y = np.empty((4, len(self.calibration_voltages)))
        self.mot2_x_cam1_x, self.mot2_x_cam1_y, self.mot2_x_cam2_x, self.mot2_x_cam2_y = np.empty((4, len(self.calibration_voltages)))
        self.mot2_y_cam1_x, self.mot2_y_cam1_y, self.mot2_y_cam2_x, self.mot2_y_cam2_y = np.empty((4, len(self.calibration_voltages)))
        self.starting_v = 75.0 * np.ones(4)

        #Init variables for applying and ROI to the camera:
        self.cam1_ROI_visiblity = True
        self.cam2_ROI_visiblity = True
        self.ROICam1_crop = None
        self.ROICam2_crop = None
        self.cam1_ROI_set = False
        self.cam2_ROI_set = False

        #Junk likely to remove:
        self.start_time = 0
        ################################################
        # Call any class methods that init should call #
        ################################################
        # connect Qt signals and slots for the UI.
        self.connectSignalsSlots()
        # Update the displayed PID settings to their startup values
        self.Update_GUI_PID()
        #init alignment arrows
        self.init_alignment_arrows()
        # Find the motors
        self.find_motors()
        # Find the cameras:
        # Actually, don't because they are found once a system is chosen.
        # self.find_cameras()
        # Setup the camera graphics view items:
        self.init_camera_views()

        ##########################
        # Setup initial GUI view #
        ##########################
        self.toggle_mightex_cam_settings_ui_vis(False)
        self.toggle_general_cam_settings_ui_vis(False)
        self.toggle_BOSON_cam_settings_ui_vis(False)
        return

    def connectSignalsSlots(self):
        """
        Connect all Qt signals and slots
        """
        self.btn_cam1_update.clicked.connect(self.update_cam1_settings)
        self.btn_cam2_update.clicked.connect(self.update_cam2_settings)
        self.btn_motor_connect.clicked.connect(self.update_motors)
        self.btn_PID_update.clicked.connect(self.update_PID)
        self.btn_load_visible.clicked.connect(self.load_visible_state)
        self.btn_load_IR.clicked.connect(self.load_IR_state)
        self.act_calibrate.triggered.connect(self.begin_calibration)
        self.actionLoad_Old_Home.triggered.connect(self.load_home)
        self.actionSave_State.triggered.connect(self.save_state)
        self.actionLoad_State.triggered.connect(self.load_state)
        self.btn_clear.clicked.connect(self.clear_pointing_plots)
        self.btn_lock.clicked.connect(self.lock_pointing)
        self.btn_Home.clicked.connect(self.define_home)
        self.btn_Align.clicked.connect(self.begin_align)
        self.pb_cam1_img_cap.clicked.connect(self.capture_cam1_img)
        self.pb_cam2_img_cap.clicked.connect(self.capture_cam2_img)
        self.cb_SystemSelection.currentIndexChanged.connect(self.find_cameras)
        self.btn_cam1_gen_ROI.clicked.connect(self.gen_cam1_ROI)
        self.btn_cam1_apply_ROI.clicked.connect(self.update_cam1_ROI_bounds)
        self.btn_cam2_gen_ROI.clicked.connect(self.gen_cam2_ROI)
        self.btn_cam2_apply_ROI.clicked.connect(self.apply_cam2_ROI)
        self.list_unlock_report.clicked.connect(self.display_unlock_report)
        self.cb_suppress_image_update.clicked.connect(self.toggle_img_display)
        self.cb_suppress_pointing_updat.clicked.connect(self.toggle_img_display)
        return

    def Update_GUI_PID(self):
        # Update the GUI with the numbers from the UpdateManager settings
        try:
           self.le_P.setText('%.2f' % (self.UpdateManager.P))
           self.le_Ti.setText('%.2f' % (self.UpdateManager.TI))
           self.le_Td.setText('%.2f' % (self.UpdateManager.TD))
        except AttributeError:
           self.le_P.setText('%.2f' % (1.0))
           self.le_Ti.setText('N/A not PID')
           self.le_Td.setText('N/A not PID')
        return

    def find_motors(self):
        # Find motors using VISA
        for dev in self.ResourceManager.list_resources():
            self.motor_model.appendRow(QtGui.QStandardItem(str(dev)))
        # Find newer mdt693b devices using Thorlabs SDK and add them to the list too
        mdt693b_dev_list = mdt.mdtListDevices()
        for dev in mdt693b_dev_list:
            self.motor_model.appendRow(QtGui.QStandardItem(str(dev)))
        self.cb_motors_1.setModel(self.motor_model)
        self.cb_motors_2.setModel(self.motor_model)
        if len(self.ResourceManager.list_resources()) > 1:
            self.cb_motors_1.setCurrentIndex(0)
            self.cb_motors_2.setCurrentIndex(1)
        else:
            self.cb_motors_1.setCurrentIndex(-1)
            self.cb_motors_1.setCurrentIndex(-1)
        return

    def init_alignment_arrows(self):
        self.Cam1_UpArrow.setRotation(90)
        self.Cam1_RightArrow.setRotation(180)
        self.Cam1_DownArrow.setRotation(-90)
        self.Cam2_RightArrow.setRotation(180)
        self.Cam2_DownArrow.setRotation(-90)
        self.Cam2_UpArrow.setRotation(90)
        self.Cam1_LeftArrow.setScale(3)
        self.Cam1_RightArrow.setScale(3)
        self.Cam1_DownArrow.setScale(3)
        self.Cam1_UpArrow.setScale(3)
        self.Cam2_LeftArrow.setScale(3)
        self.Cam2_RightArrow.setScale(3)
        self.Cam2_DownArrow.setScale(3)
        self.Cam2_UpArrow.setScale(3)
        self.gv_camera1.addItem(self.Cam1_LeftArrow)
        self.gv_camera1.addItem(self.Cam1_RightArrow)
        self.gv_camera1.addItem(self.Cam1_DownArrow)
        self.gv_camera1.addItem(self.Cam1_UpArrow)
        self.gv_camera2.getView().addItem(self.Cam2_LeftArrow)
        self.gv_camera2.getView().addItem(self.Cam2_RightArrow)
        self.gv_camera2.getView().addItem(self.Cam2_DownArrow)
        self.gv_camera2.getView().addItem(self.Cam2_UpArrow)

        self.Cam1_LeftArrow.setVisible(False)
        self.Cam1_RightArrow.setVisible(False)
        self.Cam1_DownArrow.setVisible(False)
        self.Cam1_UpArrow.setVisible(False)
        self.Cam2_LeftArrow.setVisible(False)
        self.Cam2_RightArrow.setVisible(False)
        self.Cam2_DownArrow.setVisible(False)
        self.Cam2_UpArrow.setVisible(False)
        return

    def init_camera_views(self):
        """
        Setup camera graphics view items.
        """
        self.gv_camera1.setColorMap(self.cmap)
        self.gv_camera2.setColorMap(self.cmap)
        self.gv_camera1.addItem(self.cam1_x_line)
        self.gv_camera1.addItem(self.cam1_y_line)
        self.gv_camera2.getView().addItem(self.cam2_x_line)
        self.gv_camera2.getView().addItem(self.cam2_y_line)
        # Set all COM lines invisible
        self.cam1_x_line.setVisible(False)
        self.cam1_y_line.setVisible(False)
        self.cam2_x_line.setVisible(False)
        self.cam2_y_line.setVisible(False)
        return

    # Set the camera settings buttons invisible, until a Point Lock System is chosen. Then, make visible the correct
    # set of gui buttons.
    def toggle_mightex_cam_settings_ui_vis(self, Logic):
        """
        Set visibility state of Mightex camera settings UI to true or false. i.e. visibility of settings
        relavent to only Mightex cameras.
        input: True/False
        """
        self.label.setVisible(Logic)
        self.label_2.setVisible(Logic)
        self.le_cam1_exp_time.setVisible(Logic)
        self.le_cam1_gain.setVisible(Logic)
        self.label_5.setVisible(Logic)
        self.label_6.setVisible(Logic)
        self.le_cam2_exp_time.setVisible(Logic)
        self.le_cam2_gain.setVisible(Logic)
        return

    def toggle_general_cam_settings_ui_vis(self, Logic):
        """
        Set visibility state of generally useful camera settings UI to true or false. i.e. visibility of settings
        relavent to both Mightex and Boson cameras.
        input: True/False
        """
        self.btn_cam1_update.setVisible(Logic)
        self.btn_cam2_update.setVisible(Logic)
        self.label_7.setVisible(Logic)
        self.label_8.setVisible(Logic)
        self.le_cam1_threshold.setVisible(Logic)
        self.le_cam2_threshold.setVisible(Logic)
        self.label_4.setVisible(Logic)
        self.label_3.setVisible(Logic)
        self.label_7.setVisible(Logic)
        self.cb_cam1.setVisible(Logic)
        self.cb_cam2.setVisible(Logic)
        self.label_14.setVisible(Logic)
        self.label_15.setVisible(Logic)
        self.Cam1_AvgFrames.setVisible(Logic)
        self.Cam2_AvgFrames.setVisible(Logic)
        self.label_9.setVisible(Logic)
        self.label_10.setVisible(Logic)
        self.le_cam1_max.setVisible(Logic)
        self.le_cam2_max.setVisible(Logic)
        self.btn_cam1_gen_ROI.setVisible(Logic)
        self.btn_cam1_apply_ROI.setVisible(Logic)
        self.btn_cam2_gen_ROI.setVisible(Logic)
        self.btn_cam2_apply_ROI.setVisible(Logic)
        self.pb_cam1_img_cap.setVisible(Logic)
        self.pb_cam2_img_cap.setVisible(Logic)
        return

    def toggle_BOSON_cam_settings_ui_vis(self, Logic):
        """
        Set visibility state of generally useful camera settings UI to true or false. i.e. visibility of settings
        relavent to both Mightex and Boson cameras.
        input: True/False
        """
        self.label_16.setVisible(Logic)
        self.label_17.setVisible(Logic)
        self.label_18.setVisible(Logic)
        self.label_19.setVisible(Logic)
        return

    def GUI_update_std(self, std):
       self.le_cam1_dx_std.setText("{:.5f}".format(std[0]))
       self.le_cam1_dy_std.setText("{:.5f}".format(std[1]))
       self.le_cam2_dx_std.setText("{:.5f}".format(std[2]))
       self.le_cam2_dy_std.setText("{:.5f}".format(std[3]))

    @staticmethod
    def resetHist(ref, min=0, max=255):
        """
        Given a reference to an image view, set the histogram's range and levels to min and max
        """
        assert isinstance(ref, pg.ImageView)
        ref.getHistogramWidget().setHistogramRange(min, max)
        ref.getHistogramWidget().setLevels(min, max)

    def addToPlot(data, plot, point, maxSize=100):
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

    @pyqtSlot()
    def update_cam1_settings(self):
        """
        This function updates the camera settings on camera thread 1. Inclduing selecting the camera on thread 1.
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Grab inputs from the GUI
            cam1_exp_time = float(self.le_cam1_exp_time.text())
            cam1_gain = float(self.le_cam1_gain.text())
            self.cam1_threshold = float(self.le_cam1_threshold.text())

            #Apply all updates:
            if self.cam1 is None: # first time this function is called only.
                # Instantiate a camera object as cam1.
                key = str(self.cam_model.item(self.cb_cam1.currentIndex(), 0).text())
                if 'fly' in key:
                    self.cam1 = BlackflyS(self.cam_init_dict[key])
                    if not self.cam1.serial_no in str(self.cam_init_dict[key]):
                        print("Somehow the camera initialized does not have the anticipated serial number.")
                else:
                    raise NotImplemented('Can only connect blackfly s cameras for now.')
                # move camera 1 object to camera 1 thread
                self.cam1.moveToThread(self.cam1_thread)
                # Now, connect GUI related camera signals to appropriate GUI slots.
                self.cam1.img_captured_signal.connect(self.update_cam1_img) #I might want this to come from UM instead?
                self.cam1.exposure_updated_signal.connect(self.update_cam1_exposure)
                self.cam1.gain_updated_signal.connect(self.update_cam1_gain)
                self.cam1.ROI_bounds_updated_signal.connect(self.apply_cam1_ROI)
                self.cam1.r0_updated_signal.connect(self.set_cam1_r0)
                self.cam1.cap_released_signal.connect(self.cam1_thread.quit)
                # Apply the settings directly before starting thread and thread event loop
                # Gui will autoupdate the cameras new settings by virtue of setters emitting signals back to GUI.
                self.cam1.set_gain(cam1_gain)
                self.cam1.set_exposure_time(cam1_exp_time)
                # Start the threads event loop
                # See priority options here: https://doc.qt.io/qt-6/qthread.html#Priority-enum
                self.cam1_thread.start(priority=5)
                # Setup camera view.
                self.cam1_reset = True
                self.resetHist(self.gv_camera1)
                # start the infinite event loop around acquiring images:
                self.cam1.capture_img_signal.emit()
            else: # Update settings once cam1 exists and cam1_thread is running
                key = str(self.cam_model.item(self.cb_cam1.currentIndex(), 0).text())
                if not self.cam1.serial_no in str(self.cam_init_dict[key]):
                    # User wants to change the camera on cam1_thread. So, do that.
                    # Still need to implement this change, but eitherway, we can update the settings with signals.
                    raise NotImplemented("I have not given you the ability to change camera 1 once you selected it")
                self.cam1.exposure_set_signal(cam1_exp_time)
                self.cam1.gain_set_signal(cam1_gain)
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            #TODO: Update this correctly for the Boson
            """cam1_index = int(self.cb_cam1.currentIndex())
            self.cam1_threshold = float(self.le_cam1_threshold.text())
            self.cam_list[cam1_index].update_frame()
            self.cam1_reset = True
            self.resetHist(self.gv_camera1, max=65536)"""
            pass
        else:
            print("Choose a Point Lock system first!")

    @pyqtSlot(float)
    def update_cam1_exposure(self, exposure_time):
        self.le_cam1_exp_time.setText('%.2f' % (exposure_time))
        return

    @pyqtSlot(float)
    def update_cam1_gain(self, gain):
        self.le_cam1_gain.setText('%.2f' % (gain))
        return

    @pyqtSlot(np.ndarray)
    def set_cam1_r0(self, r0):
        self.cam1_r0 = r0
        return

    @pyqtSlot(np.ndarray)
    def update_cam1_img(self, img):
        """
        Update the img displayed in cam1 image viewer object.
        """
        if not self.suppress_image_display:
            if self.cam1_reset:
                self.gv_camera1.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False,
                                         pos=self.cam1_r0)
                self.cam1_reset = False
            else:
                self.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False,
                                         pos=self.cam1_r0)

    @pyqtSlot(np.ndarray, np.ndarray, np.ndarray)
    def update_cam1_display(self, img, com, r_0):
        """
        update the image displayed for cam 1.
        """
        # Update max value of camera image:
        self.le_cam1_max.setText(str(np.max(img)))

        # Update COM crosshairs on the image:
        self.cam1_x_line.setPos(com[0])
        self.cam1_y_line.setPos(com[1])
        self.cam1_x_line.setVisible(True)
        self.cam1_y_line.setVisible(True)

        # Update the pointing position plots:
        if not self.suppress_pointing_display:
            #Redo this part.
            # self.cam1_x_plot.setData(com[0])
            # self.cam1_y_plot.setData(com[1])
            pass
        return

    def convert_img(self, img):
        pass
        return

    @pyqtSlot()
    def update_cam2_settings(self):
        """
        This function updates the camera settings on camera thread 1. Inclduing selecting the camera on thread 1.
        """
        #Use the cam1 settings above to do this once debugged:
        pass
        """if int(self.cb_SystemSelection.currentIndex()) == 1:
            cam2_index = int(self.cb_cam2.currentIndex())
            cam2_exp_time = float(self.le_cam2_exp_time.text())
            cam2_gain = float(self.le_cam2_gain.text())
            cam2_threshold = float(self.le_cam2_threshold.text())
            cam_list[cam1_index].setup_camera_for_image_acquisition()
            cam_list[cam2_index].set_gain(cam2_gain)
            cam_list[cam2_index].set_exposure_time(cam2_exp_time)
            cam_list[cam1_index].update_frame()
           self.le_cam2_exp_time.setText('%.2f' % (cam_list[cam2_index].exposure_time))
           self.le_cam2_gain.setText('%.2f' % (cam_list[cam2_index].gain / 8))
            cam2_reset = True
            resetHist(self.gv_camera2)
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            cam2_index = int(self.cb_cam2.currentIndex())
            cam2_threshold = float(self.le_cam2_threshold.text())
            cam_list[cam1_index].update_frame()
            cam2_reset = True
            resetHist(self.gv_camera2, max=65536)
        else:
            print("Choose a Point Lock system first!")"""
        return

    @pyqtSlot()
    def update_motors(self):
        """
        Instantiate motors objects and put them into motor list.
        """
        motor1_index =self.cb_motors_1.currentIndex()
        motor2_index =self.cb_motors_2.currentIndex()
        if motor1_index != motor2_index:
            self.motor_list = []
            # Garrison Updated to add "0" insideself.cb_motors_1.currentData(0)
            if "MDT693B" in self.cb_motors_1.currentData(0):
                self.motor_list.append(MDT693B_Motor(str(self.cb_motors_1.currentData(0)[2:15])))
            else:
                self.motor_list.append(
                    MDT693A_Motor(self.ResourceManager, com_port=self.cb_motors_1.currentData(0), ch1='X', ch2='Y'))
            if "MDT693B" in self.cb_motors_2.currentData(0):
                self.motor_list.append(MDT693B_Motor(str(self.cb_motors_2.currentData(0)[2:15])))
            else:
                self.motor_list.append(
                    MDT693A_Motor(self.ResourceManager, com_port=self.cb_motors_2.currentData(0), ch1='X', ch2='Y'))
        return

    @pyqtSlot()
    def clear_pointing_plots(self):
        """
        Empty the pointing plots data.
        """
        self.cam1_x = np.array([])
        self.cam1_y = np.array([])
        self.cam2_x = np.array([])
        self.cam2_y = np.array([])
        return

    @pyqtSlot()
    def begin_calibration(self):
        """
        Begin calibration. Needs updated.
        """
        self.calib_index = 0
        if len(self.motor_list) == 0:
            self.update_motors()
        self.ROICam1_Unlock.setVisible(False)
        self.ROICam2_Unlock.setVisible(False)
        self.ROICam1_Lock.setVisible(False)
        self.ROICam2_Lock.setVisible(False)
        """if int(self.cb_SystemSelection.currentIndex()) == 1:
            #Put the Mightex cameras into trigger mode.
            for cam in cam_list:
                cam.engine.update_working_mode(1, cam.serial_no)"""
        self.starting_v[0] = self.motor_list[0].ch1_v
        time.sleep(0.2)
        self.starting_v[1] = self.motor_list[0].ch2_v
        time.sleep(0.2)
        self.starting_v[2] = self.motor_list[1].ch1_v
        time.sleep(0.2)
        self.starting_v[3] = self.motor_list[1].ch2_v
        time.sleep(0.2)
        self.state = self.STATE_CALIBRATE
        return

    @pyqtSlot()
    def lock_pointing(self):
        """
        Begin to lock the pointing. Needs updated.
        """
        self.Cam1_LeftArrow.setVisible(False)
        self.Cam1_RightArrow.setVisible(False)
        self.Cam1_DownArrow.setVisible(False)
        self.Cam1_UpArrow.setVisible(False)
        self.Cam2_LeftArrow.setVisible(False)
        self.Cam2_RightArrow.setVisible(False)
        self.Cam2_DownArrow.setVisible(False)
        self.Cam2_UpArrow.setVisible(False)
        if self.btn_lock.isChecked():
            if self.cam1_thread is not None and self.cam2_thread is not None:
                self.state = self.STATE_LOCKED
                self.le_num_points.setText('100')
                self.btn_clear.click()
                self.UpdateManager._integral_ti = np.zeros(4)
                self.TimeLastUnlock = 0
                self.num_out_of_voltage_range = 1
                self.first_unlock = True  # First time since initial lock that piezos went out of bounds?
                self.LockTimeStart = time.monotonic()
                self.ROICam1_Unlock.setVisible(False)
                self.ROICam2_Unlock.setVisible(False)
                self.ROICam1_Lock.setVisible(True)
                self.ROICam2_Lock.setVisible(True)

                if self.PID:
                    if not UpdateManager.P == self.PID['P']:
                        UpdateManager.P = self.PID['P']
                    if not UpdateManager.TI == self.PID['Ti']:
                        UpdateManager.TI = self.PID['Ti']
                    if not UpdateManager.TD == self.PID['Td']:
                        UpdateManager.TD = self.PID['Td']
                if len(self.motor_list) == 0:
                    self.update_motors()
            else:
               self.btn_lock.toggle()
        else:
            self.state = self.STATE_MEASURE
            self.le_num_points.setText('100')
            self.btn_clear.click()
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
            self.ROICam1_Unlock.setVisible(True)
            self.ROICam2_Unlock.setVisible(True)
        return

    @pyqtSlot()
    def define_home(self):
        """
        Capture current pointing position and define it as home position. Save it accordingly.
        """
        if self.cam1_thread is not None and self.cam2_thread is not None:
            self.set_cam1_x = self.cam1_x[-1]
            self.set_cam1_y = self.cam1_y[-1]
            self.set_cam2_x = self.cam2_x[-1]
            self.set_cam2_y = self.cam2_y[-1]
            print('Set cam 1 x', self.set_cam1_x)
            print('Set cam 1 y', self.set_cam1_y)
            print('Set cam 2 x', self.set_cam2_x)
            print('Set cam 2 y', self.set_cam2_y)
            HomePosition = np.array([self.set_cam1_x, self.set_cam1_y, self.set_cam2_x, self.set_cam2_y])
            self.UpdateManager.set_pos = HomePosition
            if int(self.cb_SystemSelection.currentIndex()) == 1:
                np.savetxt('Most_Recent_Home.txt', HomePosition, fmt='%f')
                filename = "HomePositionStored/" + str(np.datetime64('today', 'D')) + "_Home"
                np.savetxt(filename, HomePosition, fmt='%f')
            elif int(self.cb_SystemSelection.currentIndex()) == 2:
                np.savetxt('Most_Recent_Home_IR.txt', HomePosition, fmt='%f')
                filename = "HomePositionStored/" + str(np.datetime64('today', 'D')) + "_Home_IR"
                np.savetxt(filename, HomePosition, fmt='%f')
            try:
                self.ROICam1_Unlock.setVisible(False)
                self.ROICam2_Unlock.setVisible(False)
                self.ROICam1_Lock.setVisible(False)
                self.ROICam2_Lock.setVisible(False)
            except:
                pass
            self.set_home_marker()

    @pyqtSlot()
    def load_home(self):
        """
        Open a dialogue box for the user to select a home position to load.
        TODO: make this use Qt and not TK!
        """
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename()
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            HomePosition = np.loadtxt(file_path, dtype=float)
            np.savetxt('Most_Recent_Home.txt', HomePosition, fmt='%f')
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            HomePosition = np.loadtxt(file_path, dtype=float)
            np.savetxt('Most_Recent_Home_IR.txt', HomePosition, fmt='%f')
        UpdateManager.set_pos = np.asarray(HomePosition)
        print("Set Positions:", HomePosition)
        try:
            self.ROICam1_Unlock.setVisible(False)
            self.ROICam2_Unlock.setVisible(False)
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
        except:
            pass
        self.set_home_marker()
        return

    @pyqtSlot()
    def begin_align(self):
        """
        Begin alignment mode. Need to redo.
        """
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global Cam1_LeftArrow, Cam1_RightArrow, Cam1_DownArrow, Cam1_UpArrow
        global Cam2_LeftArrow, Cam2_RightArrow, Cam2_DownArrow, Cam2_UpArrow
        global state, start_time
        global msg
        if self.state == self.STATE_ALIGN:
            self.state = self.STATE_MEASURE
            self.le_num_points.setText('100')
            self.btn_clear.click()
            self.UpdateManager.P = self.PID['P']
            self.UpdateManager.TI = self.PID['Ti']
            self.UpdateManager.TD = self.PID['Td']
            self.Update_GUI_PID()
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
            self.ROICam1_Unlock.setVisible(True)
            self.ROICam2_Unlock.setVisible(True)
            self.Cam1_LeftArrow.setVisible(False)
            self.Cam1_RightArrow.setVisible(False)
            self.Cam1_DownArrow.setVisible(False)
            self.Cam1_UpArrow.setVisible(False)
            self.Cam2_LeftArrow.setVisible(False)
            self.Cam2_RightArrow.setVisible(False)
            self.Cam2_DownArrow.setVisible(False)
            self.Cam2_UpArrow.setVisible(False)
        else:
            self.state = self.STATE_ALIGN
            self.le_num_points.setText('10')
            self.btn_clear.click()
            self.UpdateManager._integral_ti = np.zeros(4)
            self.PID = {'P': self.UpdateManager.P, 'Ti': self.UpdateManager.TI, 'Td': self.UpdateManager.TD}
            self.UpdateManager.TI = 1e20
            self.UpdateManager.P = 1
            self.UpdateManager.TD = 0
            self.Update_GUI_PID()
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
            self.ROICam1_Unlock.setVisible(True)
            self.ROICam2_Unlock.setVisible(True)
            if len(self.motor_list) == 0:
                self.update_motors()
            try:
                self.motor_list[0].ch1_v = 75.0
                self.motor_list[0].ch2_v = 75.0
                self.motor_list[1].ch1_v = 75.0
                self.motor_list[1].ch2_v = 75.0
                self.UpdateManager.V0 = np.array([self.motor_list[0].ch1_v, self.motor_list[0].ch2_v, self.motor_list[1].ch1_v,
                                             self.motor_list[1].ch2_v])
            except:
                self.UpdateManager.V0 = np.array([self.motor_list[0].ch1_v, self.motor_list[0].ch2_v, self.motor_list[1].ch1_v,
                                             self.motor_list[1].ch2_v])
                self.msg += "WARNING! Alignment should be done w/ piezos at 75 V, but motors are not connected and " \
                       "voltages are unkown!"
                self.statusbar.showMessage('{0}\tUpdate time: {1:.3f} (s)'.format(self.msg, time.time() - self.start_time))
                time.sleep(5)
                raise AssertionError("Make Piezo voltages 75.0 before aligning.")

    def set_home_marker(self):
        """
        Set the home marker on the image displays.
        """
        self.set_cam1_x, self.set_cam1_y, self.set_cam2_x, self.set_cam2_y = self.UpdateManager.set_pos
        pen = pg.mkPen(color=(255, 0, 0), width=2)
        self.ROICam1_Unlock = pg.CircleROI(pos=(self.set_cam1_y - 20, self.set_cam1_x - 20), radius=20,
                                      movable=False, rotatable=False, resizable=False, pen=pen)

        self.ROICam2_Unlock = pg.CircleROI(pos=(self.set_cam2_y - 20, self.set_cam2_x - 20), radius=20,
                                      movable=False, rotatable=False, resizable=False, pen=pen)
        self.gv_camera1.addItem(self.ROICam1_Unlock)
        self.gv_camera2.getView().addItem(self.ROICam2_Unlock)
        pen = pg.mkPen(color=(0, 255, 0), width=2)
        self.ROICam1_Lock = pg.CircleROI(pos=(self.set_cam1_y - 20, self.set_cam1_x - 20), radius=20,
                                    movable=False, rotatable=False, resizable=False, pen=pen)

        self.ROICam2_Lock = pg.CircleROI(pos=(self.set_cam2_y - 20, self.set_cam2_x - 20), radius=20,
                                    movable=False, rotatable=False, resizable=False, pen=pen)
        self.gv_camera1.addItem(self.ROICam1_Lock)
        self.gv_camera2.getView().addItem(self.ROICam2_Lock)
        self.cam1_x_PlotItem.addLine(y=self.UpdateManager.set_pos[0])
        self.cam1_y_PlotItem.addLine(y=self.UpdateManager.set_pos[1])
        self.cam2_x_PlotItem.addLine(y=self.UpdateManager.set_pos[2])
        self.cam2_y_PlotItem.addLine(y=self.UpdateManager.set_pos[3])
        if self.state == self.STATE_MEASURE:
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
            self.ROICam1_Unlock.setVisible(True)
            self.ROICam2_Unlock.setVisible(True)
        elif self.state == self.STATE_CALIBRATE:
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
            self.ROICam1_Unlock.setVisible(False)
            self.ROICam2_Unlock.setVisible(False)
        elif self.state == self.STATE_LOCKED:
            self.ROICam1_Unlock.setVisible(False)
            self.ROICam2_Unlock.setVisible(False)
            self.ROICam1_Lock.setVisible(True)
            self.ROICam2_Lock.setVisible(True)
        elif self.state == self.STATE_ALIGN:
            self.ROICam1_Unlock.setVisible(True)
            self.ROICam2_Unlock.setVisible(True)
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
        return

    @pyqtSlot()
    def find_cameras(self):
        """
        This function just populates the GUI with camera options, but should not instantiate any cameras!

        #TODO: I need to just populate lists.
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            num_cameras = 0
            """
            cam_list = []
            Commented out for testing blackfly cameras.
            # Find the Mightex cameras
            mightex_engine = MightexEngine()
            if len(mightex_engine.serial_no) == 0:
                del mightex_engine
            else:
                for i, serial_no in enumerate(mightex_engine.serial_no):
                    num_cameras += 1
                    c = MightexCamera(mightex_engine, serial_no)
                    cam_list.append(c)
                    cam_model.appendRow(QtGui.QStandardItem(c.serial_no))

            # Find blackfly s cameras:
            # Retrieve singleton reference to system object
            pyspin_system = PySpin.System.GetInstance()

            # Get current library version
            pyspin_version = pyspin_system.GetLibraryVersion()
            print('Library version: %d.%d.%d.%d' % (pyspin_version.major, pyspin_version.minor, pyspin_version.type,
                                                    pyspin_version.build))

            # Retrieve list of cameras from the system
            pyspin_cam_list = pyspin_system.GetCameras()

            num_pyspin_cameras = pyspin_cam_list.GetSize()
            num_cameras += num_pyspin_cameras

            print('Number of cameras detected: %d' % num_pyspin_cameras)

            # Finish if there are no cameras
            if num_pyspin_cameras == 0:
                # Clear camera list before releasing system
                pyspin_cam_list.Clear()

                # Release system instance
                pyspin_system.ReleaseInstance()
                return
            else:
                for i, cam in enumerate(pyspin_cam_list):
                    c = BlackflyS(cam)
                    cam_list.append(c)
                    cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
            if num_cameras == 0:
                raise DeviceNotFoundError("No visible cameras found.")
            """
            c = BlackflyS(0)
            cam_key = 'blackfly s: ' + c.serial_no
            self.cam_model.appendRow(QtGui.QStandardItem(cam_key))
            self.cam_init_dict[cam_key] = 0
            c.close()
            del c
            c = BlackflyS(1)
            cam_key = 'blackfly s: ' + c.serial_no
            self.cam_model.appendRow(QtGui.QStandardItem(cam_key))
            self.cam_init_dict[cam_key] = 1
            c.close()
            del c
            # Make appropriate Camera Settings available in GUI:
            self.toggle_BOSON_cam_settings_ui_vis(False)
            self.toggle_mightex_cam_settings_ui_vis(True)
            self.toggle_general_cam_settings_ui_vis(True)

            # Load Most Recent Calibration Matrix:
            try:
                self.UpdateManager.calibration_matrix = np.loadtxt('Most_Recent_Calibration.txt', dtype=float)
            except OSError:
                print("Hmm there seems to be no saved calibration, run calibration.")

            # Load the Most Recent Home Position:
            try:
                HomePosition = np.loadtxt('Most_Recent_Home.txt', dtype=float)
                self.UpdateManager.set_pos = np.asarray(HomePosition)
                self.set_cam1_x, self.set_cam1_y, self.set_cam2_x, self.set_cam2_y = HomePosition
                self.set_home_marker()
            except OSError:
                self.set_cam1_x = 'dummy'
                self.set_cam1_y = 'dummy'
                self.set_cam2_x = 'dummy'
                self.set_cam2_y = 'dummy'
                print("Hmm there seems to be no saved Home Position, define one before locking.")

        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # Find the Boson Cameras
            pass
            # TODO: Figure out how to correctly connect two BOSONs.
            # Boson VID and PID:
            """VID = 0x09CB
            PID = 0x4007
            device_list = list_ports.grep(r'FLIR Control')
            port_list = []
            # Find all ports associated with Bosons.
            for device in device_list:
                if device.vid == VID and device.pid == PID:
                    port = device.device
                    port_list.append(port)
            cam_list = []
            device_id = None
            for boson_port in port_list:
                c = BosonCamera(port=boson_port, device_id=device_id)
                print(boson_port)
                ffc_state = 0
                c.do_ffc()
                while ffc_state == 0:
                    ffc_state = c.get_ffc_state()
                cam_list.append(c)
                cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
                device_id = 1
            toggle_mightex_cam_settings_ui_vis(False)
            toggle_general_cam_settings_ui_vis(True)
            toggle_BOSON_cam_settings_ui_vis(True)

            # Load Most Recent Calibration Matrix:
            try:
                UpdateManager.calibration_matrix = np.loadtxt('Most_Recent_Calibration_IR.txt', dtype=float)
            except OSError:
                print("Hmm there seems to be no saved calibration, run calibration.")

            # Load the Most Recent Home Position:
            try:
                HomePosition = np.loadtxt('Most_Recent_Home_IR.txt', dtype=float)
                UpdateManager.set_pos = np.asarray(HomePosition)
                set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y = HomePosition
                set_home_marker()
            except OSError:
                set_cam1_x = 'dummy'
                set_cam1_y = 'dummy'
                set_cam2_x = 'dummy'
                set_cam2_y = 'dummy'
                print("Hmm there seems to be no saved Home Position, define one before locking.")
            """
        else:
            print("Choose a system!")
        self.btn_load_visible.setVisible(False)
        self.btn_load_IR.setVisible(False)

        self.Cam1_LeftArrow.setPos(self.set_cam1_y + 40, self.set_cam1_x - 95)
        self.Cam1_RightArrow.setPos(self.set_cam1_y - 40, self.set_cam1_x + 92)
        self.Cam1_DownArrow.setPos(self.set_cam1_y - 95, self.set_cam1_x - 40)
        self.Cam1_UpArrow.setPos(self.set_cam1_y + 95, self.set_cam1_x + 40)

        self.Cam2_LeftArrow.setPos(self.set_cam2_y + 40, self.set_cam2_x - 95)
        self.Cam2_RightArrow.setPos(self.set_cam2_y - 40, self.set_cam2_x + 92)
        self.Cam2_DownArrow.setPos(self.set_cam2_y - 95, self.set_cam2_x - 40)
        self.Cam2_UpArrow.setPos(self.set_cam2_y + 95, self.set_cam2_x + 40)

        # Update GUI List with available cameras
        self.cb_cam1.setModel(self.cam_model)
        self.cb_cam2.setModel(self.cam_model)
        return

    def service_BOSON(self, cam_index):
        """
        Handle the FFCs and NUCs of Boson.
        #TODO: I should operate in manual still and correct this to use the right command.
          See the emails with FLIR, basically the command I was calling in flirpy was really giving a 0 or 1 not the 0-4
          output option
        """
        pass
        """
        global cam_list, cam1_index, cam2_index

        if not (cam_index == cam1_index or cam_index == cam2_index):
            # If you are calling the function and not passing a cam_index that corresponds to 1 of the cameras, then
            # return without executing the function.
            return
        # Check if ffc is desired. If so, perform ffc
        if cam_list[cam_index].get_ffc_desired():
            cam_list[cam_index].do_ffc()
            num_ffc_performed = 1
            # Check to see if the FFC is complete. If not, wait until it is or initiate another FFC if requested.
            if cam_list[cam_index].get_ffc_state() != 3:
                switch = True
                while switch:
                    ffc_state = cam_list[cam_index].get_ffc_state()
                    print("FFC state:", ffc_state)
                    # 2 = FLR_BOSON_FFC_IN_PROGRESS
                    # 3 = FLR_BOSON_FFC_COMPLETE
                    if ffc_state == 3:
                        if cam_list[cam_index].get_ffc_desired():
                            cam_list[cam_index].do_ffc()
                            num_ffc_performed += 1
                            if num_ffc_performed > 3:
                                print("The camera just did 3 successive ffcs.")
                        else:
                            switch = False
                    elif ffc_state == 2:
                        pass
                    elif ffc_state == 0:
                        cam_list[cam_index].do_ffc()
                    else:
                        print(
                            "The ffc state should be in progress or complete, but it is in neither. Is FFC Mode Manual?")
                        print(ffc_state)

        # Check if NUC table switch is desired. If so, switch NUC table.
        if cam_list[cam_index].get_nuc_desired():
            cam_list[cam_index].do_nuc_table_switch

        fpa_temp = cam_list[cam_index].get_fpa_temperature()
        if fpa_temp - 68 > -5:  # permanent damage to camera at as low as 68 C. Maybe 63 C is too conservative?
            while True:
                # TODO: Can I connect to an automatic shutter?
                print("SHUTTER THE BEAM IMMEDIATELY!!!")
                Cam1_LeftArrow.setVisible(True)
                Cam1_RightArrow.setVisible(True)
                Cam1_DownArrow.setVisible(True)
                Cam1_UpArrow.setVisible(True)
                Cam2_LeftArrow.setVisible(True)
                Cam2_RightArrow.setVisible(True)
                Cam2_DownArrow.setVisible(True)
                Cam2_UpArrow.setVisible(True)
        if cam_index == cam1_index:
           self.label_17.setText(str(fpa_temp))
        elif cam_index == cam2_index:
           self.label_19.setText(str(fpa_temp))
        """
        return

    def release_hardware(self):
        """
        Release any necessary hardware.
        """
        try:
            for motor in self.motor_list:
                motor.close()
        except NameError:
            pass
        return

    @pyqtSlot()
    def manual_close(self):
        """
        What to do when app is closing? Release hardware, delete objects, and close threads. Anything else?
        """
        # Tell cam 1 to release capture instance and then close the thread
        self.cam1.close_signal.emit()
        # release any hardware
        self.release_hardware()
        return

    @pyqtSlot()
    def capture_cam1_img(self):
        """
        Save img from cam 1
        needs implemented
        """
        """
        img, _, _, _, _ = take_img(cam1_index, cam_view=0, threshold=cam1_threshold,
                                   resetView=False)
        filename = "CameraImages/" + str(np.datetime64('now', 's')) + "_cam1.txt"
        np.savetxt(filename, img, fmt='%f')
        """
        pass
        return

    @pyqtSlot()
    def capture_cam2_img(self):
        """
        Save img from cam 2
        needs implemented
        """
        """img, _, _, _, _ = take_img(cam2_index, cam_view=1, threshold=cam2_threshold,
                                   resetView=False)
        filename = "CameraImages/" + str(np.datetime64('now', 's')) + "_cam2.txt"
        np.savetxt(filename, img, fmt='%f')
        """
        pass
        return

    @pyqtSlot()
    def gen_cam1_ROI(self):
        """
        Generate a crop region interactive item to choose ROI for the camera.
        """
        if self.ROICam1_crop is None:
            self.set_cam1_x, self.set_cam1_y, _, _ = self.UpdateManager.set_pos
            pen = pg.mkPen(color=(0, 0, 0), width=2)
            self.ROICam1_crop = pg.ROI(pos=(self.set_cam1_y - 100, self.set_cam1_x - 100), size=(200, 200),
                                       movable=True, rotatable=False, resizable=True, pen=pen)
            self.gv_camera1.addItem(self.ROICam1_crop)
            self.ROICam1_crop.setVisible(True)
        else:
            if self.cam1_ROI_visiblity:
                self.ROICam1_crop.setVisible(False)
                self.cam1_ROI_visiblity = False
            else:
                self.ROICam1_crop.setVisible(True)
                self.cam1_ROI_visiblity = True
        return

    @pyqtSlot()
    def gen_cam2_ROI(self):
        """
        Generate a crop region interactive item to choose ROI for the camera.
        """
        if self.ROICam2_crop is None:
            _, _, self.set_cam2_x, self.set_cam2_y = self.UpdateManager.set_pos
            pen = pg.mkPen(color=(0, 0, 0), width=2)
            self.ROICam2_crop = pg.ROI(pos=(self.set_cam2_y - 100, self.set_cam2_x - 100), size=(200, 200),
                                       movable=True, rotatable=False, resizable=True, pen=pen)
            self.gv_camera2.addItem(self.ROICam2_crop)
            self.ROICam2_crop.setVisible(True)
        else:
            if self.cam2_ROI_visiblity:
                self.ROICam2_crop.setVisible(False)
                self.cam2_ROI_visiblity = False
            else:
                self.ROICam2_crop.setVisible(True)
                self.cam2_ROI_visiblity = True
        return

    @pyqtSlot()
    def update_cam1_ROI_bounds(self):
        """
        Update the ROI Bounds on the camera. Once ROI Bounds updated signal received, the new bounds are applied.
        """
        if self.cam1_ROI_set:
            self.cam1_ROI_set = False
            bounds = self.cam1.ROI_full_bounds
        else:
            ymin, xmin = self.ROICam1_crop.pos()
            height, width = self.ROICam1_crop.size()
            ymax = ymin + height
            xmax = xmin + width
            bounds = [xmin, xmax, ymin, ymax]
        self.cam1.ROI_bounds_set_signal.emit(bounds)
        return

    @pyqtSlot()
    def apply_cam1_ROI(self):
        """
        Apply the chosen ROI in GUI to the camera.
        """
        self.cam1.apply_ROI_signal.emit()
        return

    @pyqtSlot()
    def apply_cam2_ROI(self):
        """
        Apply the chosen ROI in GUI to the camera.
        """
        if self.cam2_ROI_set:
            self.cam2_ROI_set = False
            self.cam2_thread.cam.ROI_bounds = [0, 1024, 0, 1280]
        else:
            self.cam2_ROI_set = True
            ymin, xmin = self.ROICam2_crop.pos()
            height, width = self.ROICam2_crop.size()
            ymax = ymin + height
            xmax = xmin + width
            self.cam2_thread.cam.ROI_bounds = [xmin, xmax, ymin, ymax]
        self.cam2_thread.cam.apply_ROI()
        return

    @pyqtSlot()
    def update_PID(self):
        # Update the PID settings with numbers from the GUI
        self.UpdateManager.P = float(self.le_P.text())
        self.UpdateManager.TI = float(self.le_Ti.text())
        self.UpdateManager.TD = float(self.le_Td.text())
        self.PID = {'P': self.UpdateManager.P, 'Ti': self.UpdateManager.TI, 'Td': self.UpdateManager.TD}

        # Update the GUI with the numbers from the UpdateManager settings
        self.Update_GUI_PID()
        return

    @pyqtSlot()
    def save_state(self):
        """
        Save important parameters from the GUI for convenient loading next time.
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            UI_Settings = {'cb_SystemSelection': self.cb_SystemSelection.currentIndex()}
            UI_Settings['cb_cam1'] = self.cb_cam1.currentIndex()
            UI_Settings['le_cam1_exp_time'] = self.le_cam1_exp_time.text()
            UI_Settings['le_cam1_gain'] = self.le_cam1_gain.text()
            UI_Settings['le_cam1_threshold'] = self.le_cam1_threshold.text()
            UI_Settings['cb_cam2'] = self.cb_cam2.currentIndex()
            UI_Settings['le_cam2_exp_time'] = self.le_cam2_exp_time.text()
            UI_Settings['le_cam2_gain'] = self.le_cam2_gain.text()
            UI_Settings['le_cam2_threshold'] = self.le_cam2_threshold.text()
            UI_Settings['cb_motors_1'] = self.cb_motors_1.currentIndex()
            UI_Settings['cb_motors_2'] = self.cb_motors_2.currentIndex()
            UI_Settings['le_P'] = self.le_P.text()
            UI_Settings['le_Ti'] = self.le_Ti.text()
            UI_Settings['le_Td'] = self.le_Td.text()
            filename = r"Stored States/" + 'Most_Recent_Visible_UI_Settings'
            with open(filename + '.pkl', 'wb') as f:
                pkl.dump(UI_Settings, f)
            filename = r"Stored States/" + 'Visible_' + str(np.datetime64('today', 'D')) + "_UI_settings"
            with open(filename + '.pkl', 'wb') as f:
                pkl.dump(UI_Settings, f)
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            pass
        return

    @pyqtSlot(bool)
    def load_state(self, triggered, method = 'User_selected_file'):
        """
        Load the GUI settings saved by save_state.
        """
        if method == 'Load_Visible':
            file_path = r"Stored States/" + 'Most_Recent_Visible_UI_Settings.pkl'
        elif method == 'Load_IR':
            pass
        else:
            method = 'User_selected_file'
        if method == 'User_selected_file':
            # Stop using tk.
            print('open dialogue')
            root = tk.Tk()
            root.withdraw()
            file_path = filedialog.askopenfilename()
        with open(file_path, 'rb') as f:
            UI_Settings = pkl.load(f)
        if int(UI_Settings['cb_SystemSelection']) == 1:
            self.cb_SystemSelection.setCurrentIndex(UI_Settings['cb_SystemSelection'])

            self.cb_cam1.setCurrentIndex(UI_Settings['cb_cam1'])
            self.le_cam1_exp_time.setText(UI_Settings['le_cam1_exp_time'])
            self.le_cam1_gain.setText(UI_Settings['le_cam1_gain'])
            self.le_cam1_threshold.setText(UI_Settings['le_cam1_threshold'])
            self.update_cam1_settings()

            self.cb_cam2.setCurrentIndex(UI_Settings['cb_cam2'])
            self.le_cam2_exp_time.setText(UI_Settings['le_cam2_exp_time'])
            self.le_cam2_gain.setText(UI_Settings['le_cam2_gain'])
            self.le_cam2_threshold.setText(UI_Settings['le_cam2_threshold'])
            self.update_cam2_settings()

            self.cb_motors_1.setCurrentIndex(UI_Settings['cb_motors_1'])
            self.cb_motors_2.setCurrentIndex(UI_Settings['cb_motors_2'])
            self.update_motors()

            self.le_P.setText(UI_Settings['le_P'])
            self.le_Ti.setText(UI_Settings['le_Ti'])
            self.le_Td.setText(UI_Settings['le_Td'])
            self.update_PID()
        elif int(UI_Settings['cb_SystemSelection']) == 2:
            pass
        self.btn_load_visible.setVisible(False)
        self.btn_load_IR.setVisible(False)
        return

    @pyqtSlot()
    def display_unlock_report(self, key: str = None):
        if not isinstance(key, str):
            key = str(self.list_unlock_report.currentItem().text())
            self.user_selected_unlock_report = True
        self.label_unlock_report.setText(str(key) + "------------"
                                       + "number of successive out of bounds"
                                         " updates with less than 60 seconds "
                                         "between updates is " + str(
            self.Unlocked_report[key]["number of successive out of bounds"
                                 " updates with less than 60 seconds "
                                 "between updates"]) + "Amount of time spent outside voltage "
                                                       "range is " + str(
            self.Unlocked_report[key]["Amount of time spent outside voltage "
                                 "range"]) + "Farthest dx during this round of unlock is " +
                                       str(self.Unlocked_report[key]["Farthest dx during this round of unlock"]))
        return

    @pyqtSlot()
    def toggle_img_display(self):
        """
        turn on/off whether GUI updates image display.
        """
        if self.suppress_image_display:
            self.suppress_image_display = False
        else:
            self.suppress_image_display = True
        return

    @pyqtSlot()
    def toggle_pointing_display(self):
        """
        turn on/off whether GUI updates pointing line plots.
        """
        if self.suppress_pointing_display:
            self.suppress_pointing_display = False
        else:
            self.suppress_pointing_display = True
        return

    @pyqtSlot()
    def load_visible_state(self):
        self.load_state(method='Load_Visible')
        return

    @pyqtSlot()
    def load_IR_state(self):
        self.load_state(method='Load_IR')
        return

# Launch the code here!
from PyQt5.QtWidgets import QApplication
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    app.aboutToQuit.connect(win.manual_close)
    sys.exit(app.exec())