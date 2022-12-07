# TODO: When reporting locked/unlocked update the lock ROI symbols
# TODO: 1 Implement PID
# TODO: 1 Finish overhauling code to allow for operation with IR Cameras.
# TODO: 2 Try to shutter the TOPAS if the BOSON FPAs get too hot. Can I even control the TOPAS shutter?
# TODO: 2 try the other algorithm from the paper I found.
# TODO: 2 Make the GUI compatible with multiple screen formats.
# TODO: 2 Log the information on Grafana.
# TODO: 3 Move all print statements to the GUI.
# TODO: 2 Add GUI ability to set the averager state of the BOSON camera, and power on defaults.
# TODO: 3 It would be nice to allow the user to switch between IR and visibile system, which would require disconnecting
#  devices and reinitializing the cam_list.
# TODO: 1 I need to be able to run multiple instances of the program; so that I can run an IR and a vis instance in
#  parallel.
# TODO: I need to make sure that the older piezzo controllers still work with this code!
# TODO: Delete the align toggle button!
# TODO: Applying an ROI to the cameras fails.
# TODO: Connecting 2 cameras is failing, although connecting either camera in either camera view works fine. I do not
# think that I am simply overloading the GUI event loop, because suppressing image display does not update COM plots
# correctly??
from PyQt5.QtWidgets import QMainWindow
from Packages.pointing_ui import Ui_MainWindow

# Old imports:
import sys
import visa
import time
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThreadPool, QThread, QTimer
from Packages.camera import MightexCamera, MightexEngine, DeviceNotFoundError, BosonCamera
from Packages.camera import BlackflyS_EasyPySpin_QObject as BlackflyS
from Packages.motors import MDT693A_Motor
from Packages.motors import MDT693B_Motor
from Packages.CameraThread import CameraThread
import tkinter as tk
from tkinter import filedialog
from serial.tools import list_ports
from Packages.UpdateManager import UpdateManager
# from Packages.UpdateManager import PIDUpdateManager as UpdateManager
from Packages.UpdateManager import InsufficientInformation, update_out_of_bounds
import copy
import pickle as pkl
import gc
import matplotlib.pyplot as plt
from Thorlabs_MDT69XB_PythonSDK import MDT_COMMAND_LIB as mdt
from matplotlib.pyplot import Figure
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
        # Start update manager's thread:
        self.UpdateManager_thread = QThread()
        self.UpdateManager_thread.started.connect(lambda: print("Update Manager Thread Started."))
        self.UpdateManager_thread.finished.connect(lambda: print("Update Manager Thread has finished."))
        #  Instantiate Update Manager
        self.UpdateManager = UpdateManager()
        print("Thread that objects created by Window object live in: ", self.UpdateManager.thread())
        if self.UpdateManager.is_PID:
            self.PID = {'P': 0.5, 'Ti': 0.1, 'Td': 0}
        # move update manager to its own thread.
        self.UpdateManager.moveToThread(self.UpdateManager_thread)
        print("Update manager lives in thread, ", self.UpdateManager.thread())
        # Connect signals related to update manager.
        self.UpdateManager.connect_signals()
        self.connect_UpdateManager_signals()
        # Start the thread
        # See priority options here: https://doc.qt.io/qt-6/qthread.html#Priority-enum
        # Set priority as high. When Locking, I probably want to update the priority to time sensitive.
        self.UpdateManager_thread.start(priority=4)
        # params for Handle threading of cameras:
        self.cam1_thread = None
        self.cam2_thread = None
        self.cam1 = None
        self.cam2 = None
        # Camera Parameters
        self.cam_init_dict = {}
        self.mightex_engine = None
        # Grab the app's threadpool object to manage threads
        # self.threadpool = QThreadPool.globalInstance() Not needed yet.
        # assign item models
        self.cam_model = QtGui.QStandardItemModel()
        self.motor_model = QtGui.QStandardItemModel()
        # error dialog, unused so far.
        # self.error_dialog = QtWidgets.QErrorMessage()
        # Motors parameters
        self.motor1_index = None
        self.motor2_index = None
        self.ResourceManager = visa.ResourceManager()
        # Holds PID Dict
        self.PID = {}
        # Initialize global variables
        self.cam1_reset = True
        self.cam2_reset = True
        self.cam1_to_connect = None
        self.cam1_settings_to_set = None
        self.cam2_to_connect = None
        self.cam2_settings_to_set = None
        self.most_recent_error = ''

        # Initialize global variables for Locking
        self.Unlocked_report = {}
        self.user_selected_unlock_report = False
        self.set_cam1_x = None
        self.set_cam1_y = None
        self.set_cam2_x = None
        self.set_cam2_y = None

        # Initialize Global variables for home position markers:
        self.ROICam1_Unlock = None
        self.ROICam2_Unlock = None
        self.ROICam1_Lock = None
        self.ROICam2_Lock = None

        # Initialize global variables for tracking pointing
        self.cam1_r0 = np.array([0, 0])
        self.cam2_r0 = np.array([0, 0])
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

        # Set the com guide lines on image view
        self.cam1_x_line = pg.InfiniteLine(movable=False, angle=0)
        self.cam1_y_line = pg.InfiniteLine(movable=False, angle=90)
        self.cam2_x_line = pg.InfiniteLine(movable=False, angle=0)
        self.cam2_y_line = pg.InfiniteLine(movable=False, angle=90)
        # Init COM set position lines on COM line plots
        self.cam1_x_set_line = None
        self.cam1_y_set_line = None
        self.cam2_x_set_line = None
        self.cam2_y_set_line = None

        # Initialize global variables for calibration
        self.calib_index = 0
        self.override = False

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
        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.ping_UpdateManager)
        self.timer.start()
        return

    @pyqtSlot()
    def ping_UpdateManager(self):
        """
        Queue an event on the update manager and see how long they take to respond.
        """
        self.UpdateManager.request_ping.emit(time.monotonic())
        return

    @pyqtSlot(float)
    def report_UpdateManager_ping(self, ping_time: float):
        if ping_time > 0.005:
            print(ping_time, " is how long the Update Manager took to receive a signal to enter its slot.")
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
        self.pb_cam1_img_cap.clicked.connect(self.capture_cam1_img)
        self.pb_cam2_img_cap.clicked.connect(self.capture_cam2_img)
        self.cb_SystemSelection.currentIndexChanged.connect(self.load_system)
        self.btn_cam1_gen_ROI.clicked.connect(self.gen_cam1_ROI)
        self.btn_cam1_apply_ROI.clicked.connect(self.apply_cam1_ROI)
        self.btn_cam2_gen_ROI.clicked.connect(self.gen_cam2_ROI)
        self.btn_cam2_apply_ROI.clicked.connect(self.apply_cam2_ROI)
        self.list_unlock_report.clicked.connect(self.display_unlock_report)
        self.cb_suppress_image_update.clicked.connect(self.toggle_img_display)
        self.cb_suppress_pointing_updat.clicked.connect(self.toggle_pointing_display)
        return

    def connect_UpdateManager_signals(self):
        self.UpdateManager.update_gui_img_signal.connect(self.update_cam_img)
        self.UpdateManager.update_gui_cam_com_signal.connect(self.update_cam_com_display)
        self.UpdateManager.update_gui_new_calibration_matrix_signal.connect(self.handle_calibration_matrix_update)
        self.UpdateManager.update_gui_piezo_voltage_signal.connect(self.update_gui_with_piezo_voltages)
        self.UpdateManager.update_gui_locked_state.connect(self.confirm_lock_state)
        self.UpdateManager.update_gui_locking_update_out_of_bounds_signal.connect(self.log_unlocks)
        self.UpdateManager.update_gui_ping.connect(self.report_UpdateManager_ping)
        self.UpdateManager.request_gui_plot_calibrate_fits.connect(self.plot_calibration_fits)
        return

    @pyqtSlot(Figure)
    def plot_calibration_fits(self, fig: Figure):
        """
        Just show the axes in the main thread so that it displays properly.
        """
        fig.show()
        return

    @pyqtSlot(dict)
    def log_unlocks(self, unlock_report: dict):
        """
        Update the GUI with updated unlock reports.
        input: updated unlock report for the most recent unlock series only
        """
        key = list(unlock_report.keys())[0]
        if key not in self.Unlocked_report:
            # This is a new unlock round, so add the key to the list of reports user can select.
            self.list_unlock_report.addItem(key)
        # Eitherway, update the Unlocked_report with new info.
        self.Unlocked_report[key] = unlock_report[key]
        if not self.user_selected_unlock_report:
            # User has not selected a particular report, so display the most recent, i.e. this report.
            self.display_unlock_report(key)
        # Set the lock/unlock ROI visibility.
        self.ROICam1_Unlock.setVisible(True)
        self.ROICam2_Unlock.setVisible(True)
        self.ROICam1_Lock.setVisible(False)
        self.ROICam2_Lock.setVisible(False)
        return

    @pyqtSlot(bool)
    def confirm_lock_state(self, UpdateManager_locked_state: bool):
        """
        Make sure that the GUI agrees with the Update Manager about whether it is locked. So, that toggling
        the lock button always transistions into/out of lock as user would expect.
        UpdateManager_locked_state: True means that UpdateManager is locking
                                    False, UpdateManager is not locking.
        """
        if UpdateManager_locked_state and not self.btn_lock.isChecked():
            # Update manager is locked. So, btn should be checked. If not, toggle
            self.btn_lock.toggle()
        elif not UpdateManager_locked_state and self.btn_lock.isChecked():
            # Update manager is not locked. So, btn should not be checked. If it is checked toggle
            self.btn_lock.toggle()
        return

    @pyqtSlot(int, int, float)
    def update_gui_with_piezo_voltages(self, motor_num: int, channel_num: int, voltage: float):
        """
        Update the gui with newly set voltages. update based on motor_num and motor_channel.
        """
        if not self.cb_suppress_piezzo_update.isChecked():
            if motor_num == 1:
                if channel_num == 1:
                    self.motor1_x = self.addToPlot(self.motor1_x, self.motor1_x_plot, voltage, maxSize=int(self.le_num_points.text()))
                elif channel_num == 2:
                    self.motor1_y = self.addToPlot(self.motor1_y, self.motor1_y_plot, voltage, maxSize=int(self.le_num_points.text()))
            elif motor_num == 2:
                if channel_num == 1:
                    self.motor2_x = self.addToPlot(self.motor2_x, self.motor2_x_plot, voltage, maxSize=int(self.le_num_points.text()))
                elif channel_num == 2:
                    self.motor2_y = self.addToPlot(self.motor2_y, self.motor2_y_plot, voltage, maxSize=int(self.le_num_points.text()))
        return

    @pyqtSlot(np.ndarray)
    def handle_calibration_matrix_update(self, calib_mat: np.ndarray):
        """
        Save the calibration matrix, compare to old one, etc.
        """
        print('Calibration done!')
        print('New calibration matrix: ', calib_mat)
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Visible system.
            try:
                old_calib_mat = np.loadtxt('Most_Recent_Calibration.txt', dtype=float)
                Change = np.sqrt(np.sum(np.square(calib_mat - old_calib_mat)))
                RelativeChange = Change / np.sqrt(np.sum(np.square(old_calib_mat)))
                print('Relative change of calib matrix: ', RelativeChange)
            except OSError:
                pass
            np.savetxt('Most_Recent_Calibration.txt', calib_mat, fmt='%f')
            filename = "CalibrationMatrixStored/" + str(np.datetime64('today', 'D')) + "_Calib_mat"
            np.savetxt(filename, calib_mat, fmt='%f')
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            try:
                old_calib_mat = np.loadtxt('Most_Recent_Calibration_IR.txt', dtype=float)
                Change = np.sqrt(np.sum(np.square(calib_mat - old_calib_mat)))
                RelativeChange = Change / np.sqrt(np.sum(np.square(old_calib_mat)))
                print('Relative change of calib matrix: ', RelativeChange)
            except OSError:
                pass
            # Boson cameras
            np.savetxt('Most_Recent_Calibration_IR.txt', calib_mat, fmt='%f')
            filename = "CalibrationMatrixStored/" + str(np.datetime64('today', 'D')) + "_Calib_mat_IR"
            np.savetxt(filename, calib_mat, fmt='%f')

        # Update Lock circles accordingly.
        self.ROICam1_Lock.setVisible(False)
        self.ROICam2_Lock.setVisible(False)
        self.ROICam1_Unlock.setVisible(True)
        self.ROICam2_Unlock.setVisible(True)
        return

    def Update_GUI_PID(self):
        # Update the GUI with the numbers from the UpdateManager settings
        if self.UpdateManager.is_PID:
            self.le_P.setText('%.2f' % (self.PID['P']))
            self.le_Ti.setText('%.2f' % (self.PID['Ti']))
            self.le_Td.setText('%.2f' % (self.PID['Td']))
        else:
            self.le_P.setText('%.2f' % (1))
            self.le_Ti.setText('N/A not PID')
            self.le_Td.setText('N/A not PID')
        return

    def find_motors(self):
        """
        Find available motors and populate the GUI list with them.
        """
        num_motors = 0
        if self.ResourceManager is None:
            self.ResourceManager = visa.ResourceManager()
        # Find motors using VISA
        for dev in self.ResourceManager.list_resources():
            num_motors += 1
            self.motor_model.appendRow(QtGui.QStandardItem(str("MDT693A?" + dev)))
        self.ResourceManager.close()
        del self.ResourceManager
        self.ResourceManager = None

        # Find newer mdt693b devices using Thorlabs SDK and add them to the list too
        mdt693b_dev_list = mdt.mdtListDevices()
        for dev in mdt693b_dev_list:
            num_motors += 1
            self.motor_model.appendRow(QtGui.QStandardItem(str(dev)))
        self.cb_motors_1.setModel(self.motor_model)
        self.cb_motors_2.setModel(self.motor_model)
        if num_motors > 1:
            self.cb_motors_1.setCurrentIndex(0)
            self.cb_motors_2.setCurrentIndex(1)
        else:
            self.cb_motors_1.setCurrentIndex(-1)
            self.cb_motors_1.setCurrentIndex(-1)
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

    @pyqtSlot(str, bool)
    def report_thread_updates(self, threadname: str, thread_starting: bool):
        """
        print a statement reflecting whether the thread has started (True) or is stopping (False)
        """
        if thread_starting:
            print(threadname + ' thread starting.')
        else:
            print(threadname + ' thread finished')
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

    def update_gui_std(self, cam_num: int):
        """
        Update the GUI to display a new std. This is of the com data that is displaying on the plots.
        updates particular values based on cam_num
        """
        if cam_num == 1:
            self.le_cam1_dx_std.setText("{:.5f}".format(self.cam1_x.std()))
            self.le_cam1_dy_std.setText("{:.5f}".format(self.cam1_y.std()))
        elif cam_num == 2:
            self.le_cam2_dx_std.setText("{:.5f}".format(self.cam2_x.std()))
            self.le_cam2_dy_std.setText("{:.5f}".format(self.cam2_y.std()))
        return

    @staticmethod
    def resetHist(ref, min=0, max=255):
        """
        Given a reference to an image view, set the histogram's range and levels to min and max
        """
        assert isinstance(ref, pg.ImageView)
        ref.getHistogramWidget().setHistogramRange(min, max)
        ref.getHistogramWidget().setLevels(min, max)
        return

    @staticmethod
    def addToPlot(data, plot, point, maxSize=100):
        """
        Scrolling plot with a maximum size
        """
        if data.size == maxSize:
            data = np.roll(data, -1)
            data[-1] = point
        elif data.size < maxSize:
            data = np.append(data, point)
        elif data.size > maxSize:
            start_ind = data.size - maxSize + 1
            data = data[start_ind:]
            data = np.append(data, point)
        plot.setData(data)
        return data

    def find_cameras(self):
        """
        This function finds cameras relevent to system selection type (IR or vis). Populate a list of cameras in GUI
        for user to choose from, but do not instantiate any objects!
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            num_cameras = 0
            # Find the Mightex cameras
            """
            # TODO: Update code to work with Mightex Cameras again
            try:
                mightex_engine = MightexEngine()
                if len(mightex_engine.serial_no) == 0:
                    del mightex_engine
                else:
                    for i, serial_no in enumerate(mightex_engine.serial_no):
                        cam_key = 'Mightex: ' + str(serial_no)
                        self.cam_model.appendRow(QtGui.QStandardItem(cam_key))
                        self.cam_init_dict[cam_key] = serial_no
                        num_cameras += 1
                    del mightex_engine
            except UnicodeError:
                # TODO: Mightex Engine is having problems decoding returned S/N. Debug.
                pass"""
            # Find blackfly s cameras
            try:
                # For now manually add the serial number and do not confirm that the camera is openable.
                # c = BlackflyS(0)
                num_cameras += 1
                # cam_key = 'blackfly s: ' + c.serial_no
                cam_key = 'blackfly s: ' + str(0)
                self.cam_model.appendRow(QtGui.QStandardItem(cam_key))
                self.cam_init_dict[cam_key] = 0
                # c.close()
                # del c
            except:
                pass
            try:
                # For now manually add the serial number and do not confirm that the camera is openable.
                # c = BlackflyS(1)
                num_cameras += 1
                # cam_key = 'blackfly s: ' + c.serial_no
                cam_key = 'blackfly s: ' + str(1)
                self.cam_model.appendRow(QtGui.QStandardItem(cam_key))
                self.cam_init_dict[cam_key] = 1
                # c.close()
                # del c
            except:
                # Well no camera at index 1.
                pass
            # Are there cameras?
            if num_cameras == 0:
                raise DeviceNotFoundError("No visible cameras found.")
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
            """
        else:
            print("Choose a system!")
        # Update GUI List with available cameras
        self.cb_cam1.setModel(self.cam_model)
        self.cb_cam2.setModel(self.cam_model)
        return

    def load_recent_home(self):
        """
        Try to load the most recent home position based on system chosen (vis vs IR).
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Load the Most Recent Home Position:
            try:
                HomePosition = np.loadtxt('Most_Recent_Home.txt', dtype=float)
                self.UpdateManager.request_set_home_signal.emit(np.asarray(HomePosition))
                self.set_cam1_x, self.set_cam1_y, self.set_cam2_x, self.set_cam2_y = HomePosition
                self.set_home_marker()
            except OSError:
                self.set_cam1_x = 'dummy'
                self.set_cam1_y = 'dummy'
                self.set_cam2_x = 'dummy'
                self.set_cam2_y = 'dummy'
                print("Hmm there seems to be no saved Home Position, define one before locking.")

        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # Load the Most Recent Home Position:
            try:
                HomePosition = np.loadtxt('Most_Recent_Home_IR.txt', dtype=float)
                self.UpdateManager.request_set_home_signal.emit(np.asarray(HomePosition))
                self.set_cam1_x, self.set_cam1_y, self.set_cam2_x, self.set_cam2_y = HomePosition
                self.set_home_marker()
            except OSError:
                self.set_cam1_x = 'dummy'
                self.set_cam1_y = 'dummy'
                self.set_cam2_x = 'dummy'
                self.set_cam2_y = 'dummy'
                print("Hmm there seems to be no saved Home Position, define one before locking.")
        else:
            print("Choose a system!")
        return

    def load_recent_calibration_matrix(self):
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Load Most Recent Calibration Matrix:
            try:
                self.UpdateManager.request_set_calibration_matrix.emit(np.asarray(np.loadtxt(
                    'Most_Recent_Calibration.txt', dtype=float)))
            except OSError:
                print("Hmm there seems to be no saved calibration, run calibration.")
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # Load Most Recent Calibration Matrix:
            try:
                self.UpdateManager.request_set_calibration_matrix.emit(
                    np.asarray(np.loadtxt('Most_Recent_Calibration_IR.txt', dtype=float)))
            except OSError:
                print("Hmm there seems to be no saved calibration, run calibration.")
        else:
            print("Choose a system!")
        return

    def update_gui_after_system_chosen(self):
        """
        Make any updates to the GUI based on a system selection (IR vs. visible).
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Make appropriate Camera Settings available in GUI:
            self.toggle_BOSON_cam_settings_ui_vis(False)
            self.toggle_mightex_cam_settings_ui_vis(True)
            self.toggle_general_cam_settings_ui_vis(True)
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # Make appropriate Camera Settigns available in GUI
            self.toggle_mightex_cam_settings_ui_vis(False)
            self.toggle_general_cam_settings_ui_vis(True)
            self.toggle_BOSON_cam_settings_ui_vis(True)
        else:
            print("Choose a system!")
        self.btn_load_visible.setVisible(False)
        self.btn_load_IR.setVisible(False)
        return

    @pyqtSlot()
    def load_system(self):
        """
        Once a system selection is made, this function is called and finds cameras and attempts to load recent home and
        calibration matrix.
        """
        self.find_cameras()
        self.load_recent_home()
        self.load_recent_calibration_matrix()
        self.update_gui_after_system_chosen()
        return

    def connect_camera_signals(self, cam_number: int):
        if cam_number == 1:
            self.cam1.r0_updated_signal.connect(lambda r0: self.UpdateManager.request_update_r0_signal.emit(1, r0))
            self.cam1.img_captured_signal.connect(lambda img, time_stamp: self.UpdateManager.process_img(1, img,
                                                                                                         time_stamp))
            self.cam1.exposure_updated_signal.connect(lambda exp: self.update_cam_exposure(1, exp))
            self.cam1.gain_updated_signal.connect(lambda gain: self.update_cam_gain(1, gain))
            self.cam1.ROI_bounds_updated_signal.connect(self.apply_cam1_ROI)
            self.cam1.r0_updated_signal.connect(lambda r0: self.set_cam_r0(1, r0))
            self.cam1.destroyed.connect(lambda args: self.reconnect_cameras(1))
            self.cam1.destroyed.connect(lambda args: self.UpdateManager.request_update_num_cameras_connected_signal(-1))
            self.cam1.ROI_applied.connect(lambda flag: self.update_cam_ROI_set(flag, cam_num=1))
            if self.UpdateManager.is_PID:
                self.cam1.exposure_updated_signal.connect(lambda exp:
                                                          self.UpdateManager.
                                                          request_update_cam_exposure_time.connect(1, exp))
        elif cam_number == 2:
            self.cam2.r0_updated_signal.connect(lambda r0: self.UpdateManager.request_update_r0_signal.emit(2, r0))
            self.cam2.img_captured_signal.connect(lambda img, time_stamp: self.UpdateManager.process_img(2, img,
                                                                                                         time_stamp))
            self.cam2.exposure_updated_signal.connect(lambda exp: self.update_cam_exposure(2, exp))
            self.cam2.gain_updated_signal.connect(lambda gain: self.update_cam_gain(2, gain))
            self.cam2.ROI_bounds_updated_signal.connect(self.apply_cam2_ROI)
            self.cam2.r0_updated_signal.connect(lambda r0: self.set_cam_r0(2, r0))
            self.cam2.destroyed.connect(lambda args: self.reconnect_cameras(2))
            self.cam2.destroyed.connect(lambda args: self.UpdateManager.request_update_num_cameras_connected_signal(-1))
            self.cam2.ROI_applied.connect(lambda flag: self.update_cam_ROI_set(flag, cam_num=2))
            if self.UpdateManager.is_PID:
                self.cam2.exposure_updated_signal.connect(lambda exp:
                                                          self.UpdateManager.
                                                          request_update_cam_exposure_time.connect(2, exp))
        return

    @pyqtSlot(int)
    def reconnect_cameras(self, cam_number: int):
        """
        Reconnect cameras. This function is called when changing a camera on a particular thread. So, if the user
        updates camera settings, including the camera selected, then this is called.
        This function connects a new camera and updates its settings according to user selected settings.
        # TODO: Known bug! This function was designed to be called on cam_object deletion, but it is not being called at
            that time. Instead, the user needs to hit update again, and the try/except statements call this. This does
            allow the user to do the desired camera swap, but requires a second attempt—buggy.
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            if cam_number == 1:
                key = self.cam1_to_connect
                if 'fly' in key:
                    self.cam1 = BlackflyS(self.cam_init_dict[key])
                    if self.cam1.serial_no not in str(self.cam_init_dict[key]):
                        print("Somehow the camera initialized does not have the anticipated serial number.")
                elif "Mightex" in key:
                    if self.mightex_engine is None:
                        self.mightex_engine = MightexEngine()
                    self.cam1 = MightexCamera(self.mightex_engine, self.cam_init_dict[key])
                else:
                    raise NotImplemented('Choose a supported camera type.')
                self.UpdateManager.request_update_num_cameras_connected_signal.emit(1)
                # Apply the settings directly before starting thread and thread event loop
                # Gui will autoupdate the cameras new settings by virtue of setters emitting signals back to GUI.
                self.cam1.set_gain(self.cam1_settings_to_set['gain'])
                self.cam1.set_exposure_time(self.cam1_settings_to_set['exposure'])
                # move camera 1 object to camera 1 thread
                self.cam1.moveToThread(self.cam1_thread)
                # Now, connect GUI related camera signals to appropriate GUI slots.
                self.cam1.connect_signals()
                self.connect_camera_signals(1)
                # Setup camera view.
                self.cam1_reset = True
                self.resetHist(self.gv_camera1)
                # start the infinite event loop around acquiring images:
                self.cam1.timer.start()
            elif cam_number == 2:
                key = self.cam2_to_connect
                if 'fly' in key:
                    self.cam2 = BlackflyS(self.cam_init_dict[key])
                    if self.cam2.serial_no not in str(self.cam_init_dict[key]):
                        print("Somehow the camera initialized does not have the anticipated serial number.")
                elif "Mightex" in key:
                    if self.mightex_engine is None:
                        self.mightex_engine = MightexEngine()
                    self.cam2 = MightexCamera(self.mightex_engine, self.cam_init_dict[key])
                else:
                    raise NotImplemented('Choose a supported camera type.')
                self.UpdateManager.request_update_num_cameras_connected_signal.emit(1)
                # Apply the settings directly before starting thread and thread event loop
                # Gui will autoupdate the cameras new settings by virtue of setters emitting signals back to GUI.
                self.cam2.set_gain(self.cam2_settings_to_set['gain'])
                self.cam2.set_exposure_time(self.cam2_settings_to_set['exposure'])
                # move camera 2 object to camera 2 thread
                self.cam2.moveToThread(self.cam2_thread)
                # Now, connect GUI related camera signals to appropriate GUI slots.
                self.cam2.connect_signals()
                self.connect_camera_signals(2)
                # Setup camera view.
                self.cam2_reset = True
                self.resetHist(self.gv_camera2)
                # start the infinite event loop around acquiring images:
                self.cam2.timer.start()
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # TODO: Update this correctly for the Boson
            pass
        else:
            print("Choose a Point Lock system first!")
        return

    @pyqtSlot(float)
    def update_cam_exposure(self, cam_num: int, exposure_time: float):
        if cam_num == 1:
            self.le_cam1_exp_time.setText('%.2f' % (exposure_time))
        elif cam_num == 2:
            self.le_cam2_exp_time.setText('%.2f' % (exposure_time))
        return

    @pyqtSlot(float)
    def update_cam_gain(self, cam_num: int, gain: float):
        if cam_num == 1:
            self.le_cam1_gain.setText('%.2f' % (gain))
        elif cam_num == 2:
            self.le_cam2_gain.setText('%.2f' % (gain))
        return

    @pyqtSlot(np.ndarray)
    def set_cam_r0(self, cam_num: int, r0: np.ndarray):
        if cam_num == 1:
            self.cam1_r0 = r0
        elif cam_num == 2:
            self.cam2_r0 = r0
        return

    @pyqtSlot(int, np.ndarray)
    def update_cam_img(self, cam_num: int,  img: np.ndarray):
        """
        Update the img displayed in cam1 image viewer object.
        """
        # TODO: Do this faster?
        # TODO: Instead of checking if image display is suppressed, if image display suppressed, then destroy
        # the connection to this function in the first place. Same for other suppressed plottings.
        if not self.suppress_image_display:
            if cam_num == 1:
                if self.cam1_reset:
                    self.gv_camera1.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False,
                                             pos=self.cam1_r0)
                    self.cam1_reset = False
                else:
                    self.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False,
                                             pos=self.cam1_r0)
                # Update max value of camera image:
                self.le_cam1_max.setText(str(np.max(img)))
            elif cam_num == 2:
                if self.cam2_reset:
                    self.gv_camera2.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False,
                                             pos=self.cam2_r0)
                    self.cam2_reset = False
                else:
                    self.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False,
                                             pos=self.cam2_r0)
                # Update max value of camera image:
                self.le_cam2_max.setText(str(np.max(img)))
        return

    @pyqtSlot(int, np.ndarray)
    def update_cam_com_display(self, cam_num: int, cam_com: np.ndarray):
        """
        Set the crosshairs for pointing on image display and also plot the pointing information as time series.
        """
        if cam_num == 1:
            if not self.suppress_image_display:
                # Update COM crosshairs on the image:
                self.cam1_x_line.setPos(cam_com[0])
                self.cam1_y_line.setPos(cam_com[1])
                self.cam1_x_line.setVisible(True)
                self.cam1_y_line.setVisible(True)

            # Update the pointing position plots:
            if not self.suppress_pointing_display:
                self.cam1_x = self.addToPlot(self.cam1_x, self.cam1_x_plot, cam_com[0],
                                             maxSize=int(self.le_num_points.text()))
                self.cam1_y = self.addToPlot(self.cam1_y, self.cam1_y_plot, cam_com[1],
                                             maxSize=int(self.le_num_points.text()))
                self.update_gui_std(1)
        elif cam_num == 2:
            if not self.suppress_image_display:
                # Update COM crosshairs on the image:
                self.cam2_x_line.setPos(cam_com[0])
                self.cam2_y_line.setPos(cam_com[1])
                self.cam2_x_line.setVisible(True)
                self.cam2_y_line.setVisible(True)

            # Update the pointing position plots:
            if not self.suppress_pointing_display:
                self.cam2_x = self.addToPlot(self.cam2_x, self.cam2_x_plot, cam_com[0],
                                             maxSize=int(self.le_num_points.text()))
                self.cam2_y = self.addToPlot(self.cam2_y, self.cam2_y_plot, cam_com[1],
                                             maxSize=int(self.le_num_points.text()))
                self.update_gui_std(2)
        return

    def convert_img(self, img):
        pass
        return

    @pyqtSlot()
    def update_cam1_settings(self):
        """
        This function updates the camera settings on camera thread 1. Including selecting the camera on thread 1.
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Grab inputs from the GUI
            cam1_exp_time = float(self.le_cam1_exp_time.text())
            cam1_gain = float(self.le_cam1_gain.text())
            cam1_threshold = np.round(float(self.le_cam1_threshold.text())).astype('uint8')
            self.le_cam1_threshold.setText(str(cam1_threshold))
            self.UpdateManager.request_set_camera_threshold_signal.emit(1, cam1_threshold)

            # Apply all updates:
            if self.cam1_thread is None:  # first time this function is called only.
                self.cam1_thread = QThread()
                self.cam1_thread.finished.connect(lambda: print("Camera 1 thread has finished."))
                self.cam1_thread.started.connect(lambda: print("Camera 1 thread has started."))
                # Instantiate a camera object as cam1.
                key = str(self.cam_model.item(self.cb_cam1.currentIndex(), 0).text())
                if 'fly' in key:
                    self.cam1 = BlackflyS(self.cam_init_dict[key])
                    if self.cam1.serial_no not in str(self.cam_init_dict[key]):
                        print("Somehow the camera initialized does not have the anticipated serial number.")
                elif "Mightex" in key:
                    if self.mightex_engine is None:
                        self.mightex_engine = MightexEngine()
                    self.cam1 = MightexCamera(self.mightex_engine, self.cam_init_dict[key])
                else:
                    raise NotImplemented('Choose a supported camera type.')
                self.UpdateManager.request_update_num_cameras_connected_signal.emit(1)
                # move camera 1 object to camera 1 thread
                self.cam1.moveToThread(self.cam1_thread)
                # Now, connect GUI related camera signals to appropriate GUI slots.
                self.cam1.connect_signals()
                self.connect_camera_signals(1)
                # Start the threads event loop
                # See priority options here: https://doc.qt.io/qt-6/qthread.html#Priority-enum
                self.cam1_thread.start(priority=4)
                # Gui will autoupdate the cameras new settings by virtue of setters emitting signals back to GUI.
                self.cam1.gain_set_signal.emit(cam1_gain)
                self.cam1.exposure_set_signal.emit(cam1_exp_time)
                # Setup camera view.
                self.cam1_reset = True
                self.resetHist(self.gv_camera1)
                # start the infinite event loop around acquiring images:
                self.cam1.timer.start()
            else:  # Update settings once cam1 exists and cam1_thread is running
                key = str(self.cam_model.item(self.cb_cam1.currentIndex(), 0).text())
                self.cam1_settings_to_set = {'exposure': cam1_exp_time, 'gain': cam1_gain}
                self.cam1_to_connect = key
                try:
                    if self.cam1.serial_no not in str(self.cam_init_dict[key]):
                        # User wants to change the camera on cam1_thread. So, do that.
                        self.cam1.close_signal.emit(False)  # False flag does not close the thread.
                        return
                    self.cam1.exposure_set_signal.emit(cam1_exp_time)
                    self.cam1.gain_set_signal.emit(cam1_gain)
                except RuntimeError as e:
                    if 'has been deleted' in str(e):
                        # This error is being thrown, because camera1 does not currently exist. So, call the reconnect
                        # Camera function, which should reconnect the camera to connect with setttigns to set.
                        self.reconnect_cameras(cam_number=1)
                    # Otherwise, I do not know why this is happening. So, raise the error as is.
                    raise RuntimeError(str(e))
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # TODO: Update this correctly for the Boson
            """cam1_index = int(self.cb_cam1.currentIndex())
            self.cam1_threshold = float(self.le_cam1_threshold.text())
            self.cam_list[cam1_index].update_frame()
            self.cam1_reset = True
            self.resetHist(self.gv_camera1, max=65536)"""
            pass
        else:
            print("Choose a Point Lock system first!")
        return

    @pyqtSlot()
    def update_cam2_settings(self):
        """
        This function updates the camera settings on camera thread 2. Including selecting the camera on thread 2.
        """
        if int(self.cb_SystemSelection.currentIndex()) == 1:
            # Grab inputs from the GUI
            cam2_exp_time = float(self.le_cam2_exp_time.text())
            cam2_gain = float(self.le_cam2_gain.text())
            cam2_threshold = np.round(float(self.le_cam2_threshold.text())).astype('uint8')
            self.le_cam2_threshold.setText(str(cam2_threshold))
            self.UpdateManager.request_set_camera_threshold_signal.emit(2, cam2_threshold)

            # Apply all updates:
            if self.cam2_thread is None:  # first time this function is called only.
                self.cam2_thread = QThread()
                self.cam2_thread.finished.connect(lambda: print("Camera 2 thread has finished."))
                self.cam2_thread.started.connect(lambda: print("Camera 2 thread has started."))
                # Instantiate a camera object as cam1.
                key = str(self.cam_model.item(self.cb_cam2.currentIndex(), 0).text())
                if 'fly' in key:
                    self.cam2 = BlackflyS(self.cam_init_dict[key])
                    if self.cam2.serial_no not in str(self.cam_init_dict[key]):
                        print("Somehow the camera initialized does not have the anticipated serial number.")
                elif "Mightex" in key:
                    if self.mightex_engine is None:
                        self.mightex_engine = MightexEngine()
                    self.cam2 = MightexCamera(self.mightex_engine, self.cam_init_dict[key])
                else:
                    raise NotImplemented('Choose a supported camera type.')
                self.UpdateManager.request_update_num_cameras_connected_signal.emit(1)
                # move camera 2 object to camera 2 thread
                self.cam2.moveToThread(self.cam2_thread)
                # Now, connect GUI related camera signals to appropriate GUI slots.
                self.cam2.connect_signals()
                self.connect_camera_signals(2)
                # Start the threads event loop
                # See priority options here: https://doc.qt.io/qt-6/qthread.html#Priority-enum
                self.cam2_thread.start(priority=4)
                # Gui will autoupdate the cameras new settings by virtue of setters emitting signals back to GUI.
                self.cam2.gain_set_signal.emit(cam2_gain)
                self.cam2.exposure_set_signal.emit(cam2_exp_time)
                # Setup camera view.
                self.cam2_reset = True
                self.resetHist(self.gv_camera2)
                # start the infinite event loop around acquiring images:
                self.cam2.timer.start()
            else:  # Update settings once cam1 exists and cam1_thread is running
                key = str(self.cam_model.item(self.cb_cam2.currentIndex(), 0).text())
                self.cam2_to_connect = key
                self.cam2_settings_to_set = {'exposure': cam2_exp_time, 'gain': cam2_gain}
                try:
                    if self.cam2.serial_no not in str(self.cam_init_dict[key]):
                        # User wants to change the camera on cam2_thread. So, do that.
                        self.cam2.close_signal.emit(False) # False flag does not close the thread.
                        return
                    self.cam2.exposure_set_signal.emit(cam2_exp_time)
                    self.cam2.gain_set_signal.emit(cam2_gain)
                except RuntimeError as e:
                    if 'has been deleted' in str(e):
                        # This error is being thrown, because camera1 does not currently exist. So, call the reconnect
                        # Camera function, which should reconnect the camera to connect with setttigns to set.
                        self.reconnect_cameras(cam_number=2)
                    # Otherwise, I do not know why this is happening. So, raise the error as is.
                    raise RuntimeError(str(e))
        elif int(self.cb_SystemSelection.currentIndex()) == 2:
            # TODO: Update this correctly for the Boson
            """cam1_index = int(self.cb_cam1.currentIndex())
            self.cam1_threshold = float(self.le_cam1_threshold.text())
            self.cam_list[cam1_index].update_frame()
            self.cam1_reset = True
            self.resetHist(self.gv_camera1, max=65536)"""
            pass
        else:
            print("Choose a Point Lock system first!")
        return

    @pyqtSlot()
    def update_motors(self):
        """
        Tell the update manager to connect the motors.
        """
        if self.cb_motors_1.currentData(0) != self.cb_motors_2.currentData(0):
            # Garrison Updated to add "0" insideself.cb_motors_1.currentData(0)
            if "MDT693B" in self.cb_motors_1.currentData(0):
                self.UpdateManager.request_connect_motor_signal.emit(1, self.cb_motors_1.currentData(0))
            else:
                self.UpdateManager.request_connect_motor_signal.emit(1, str(self.cb_motors_1.currentData(0)))
            if "MDT693B" in self.cb_motors_2.currentData(0):
                self.UpdateManager.request_connect_motor_signal.emit(2, self.cb_motors_2.currentData(0))
            else:
                self.UpdateManager.request_connect_motor_signal.emit(2, str(self.cb_motors_2.currentData(0)))
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
        Begin calibration. Update GUI and tell Update Manager to calibrate.
        """
        self.ROICam1_Unlock.setVisible(False)
        self.ROICam2_Unlock.setVisible(False)
        self.ROICam1_Lock.setVisible(False)
        self.ROICam2_Lock.setVisible(False)
        self.UpdateManager.request_calibrate_signal.emit()
        return

    @pyqtSlot()
    def lock_pointing(self):
        """
        Begin to lock the pointing. Update the GUI and tell UpdateManager to lock.
        """
        if self.btn_lock.isChecked():
            if self.cam1_thread is not None and self.cam2_thread is not None:
                # Start locking
                # Update GUI
                self.ROICam1_Unlock.setVisible(False)
                self.ROICam2_Unlock.setVisible(False)
                self.ROICam1_Lock.setVisible(True)
                self.ROICam2_Lock.setVisible(True)
                """if self.PID:
                    # TODO: Need to implement this.
                    self.UpdateManager._integral_ti = np.zeros(4)
                    if not UpdateManager.P == self.PID['P']:
                        UpdateManager.P = self.PID['P']
                    if not UpdateManager.TI == self.PID['Ti']:
                        UpdateManager.TI = self.PID['Ti']
                    if not UpdateManager.TD == self.PID['Td']:
                        UpdateManager.TD = self.PID['Td']"""
                # Tell UpdateManager to lock.
                self.UpdateManager.request_toggle_lock_signal.emit(True)
            else:
                self.btn_lock.toggle()
        else:
            # Update GUI
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
            self.UpdateManager.request_set_home_signal.emit(HomePosition)
            self.set_home_marker()
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
        return

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
        self.UpdateManager.request_set_home_signal.emit(np.asarray(HomePosition))
        self.set_cam1_x, self.set_cam1_y, self.set_cam2_x, self.set_cam2_y = HomePosition
        self.set_home_marker()
        print("Set Positions:", HomePosition)
        try:
            self.ROICam1_Unlock.setVisible(False)
            self.ROICam2_Unlock.setVisible(False)
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
        except:
            pass
        return

    def set_home_marker(self):
        """
        Set the home marker on the image displays.
        """
        pen = pg.mkPen(color=(255, 0, 0), width=2)
        # Update the ROI Circles on the camera view with the new set points.
        if self.ROICam1_Unlock is not None:
            # Remove the old one and delete the old one.
            self.gv_camera1.removeItem(self.ROICam1_Unlock)
            del self.ROICam1_Unlock
        # Make the updated ROI's for unlock display
        self.ROICam1_Unlock = pg.CircleROI(pos=(self.set_cam1_y - 20, self.set_cam1_x - 20), radius=20,
                                      movable=False, rotatable=False, resizable=False, pen=pen)

        if self.ROICam2_Unlock is not None:
            # Remove the old one and delete the old one.
            self.gv_camera2.getView().removeItem(self.ROICam2_Unlock)
            del self.ROICam2_Unlock
        # Make the updated ROI's for unlock display
        self.ROICam2_Unlock = pg.CircleROI(pos=(self.set_cam2_y - 20, self.set_cam2_x - 20), radius=20,
                                      movable=False, rotatable=False, resizable=False, pen=pen)
        # Add the updated ROIS to the camera view items.
        self.gv_camera1.addItem(self.ROICam1_Unlock)
        self.gv_camera2.getView().addItem(self.ROICam2_Unlock)
        pen = pg.mkPen(color=(0, 255, 0), width=2)
        if self.ROICam1_Lock is not None:
            # Remove the old one and delete the old one.
            self.gv_camera1.removeItem(self.ROICam1_Lock)
            del self.ROICam1_Lock
        # Make updated Lock ROI.
        self.ROICam1_Lock = pg.CircleROI(pos=(self.set_cam1_y - 20, self.set_cam1_x - 20), radius=20,
                                    movable=False, rotatable=False, resizable=False, pen=pen)
        if self.ROICam2_Lock is not None:
            # Remove the old one and delete the old one.
            self.gv_camera2.getView().removeItem(self.ROICam2_Lock)
            del self.ROICam2_Lock
        # Make updated Lock ROI.
        self.ROICam2_Lock = pg.CircleROI(pos=(self.set_cam2_y - 20, self.set_cam2_x - 20), radius=20,
                                    movable=False, rotatable=False, resizable=False, pen=pen)
        # Add the updated Lock icons to the camera views
        self.gv_camera1.addItem(self.ROICam1_Lock)
        self.gv_camera2.getView().addItem(self.ROICam2_Lock)

        # Update the COM line plots to show the target position as an infinite line item.
        if self.cam1_x_set_line is None:
            self.cam1_x_set_line = self.cam1_x_PlotItem.addLine(y=self.set_cam1_x)
        else:
            self.cam1_x_set_line.setValue(self.set_cam1_x)
        if self.cam1_y_set_line is None:
            self.cam1_y_set_line = self.cam1_y_PlotItem.addLine(y=self.set_cam1_y)
        else:
            self.cam1_y_set_line.setValue(self.set_cam1_y)
        if self.cam2_x_set_line is None:
            self.cam2_x_set_line = self.cam2_x_PlotItem.addLine(y=self.set_cam2_x)
        else:
            self.cam2_x_set_line.setValue(self.set_cam2_x)
        if self.cam2_y_set_line is None:
            self.cam2_y_set_line = self.cam2_y_PlotItem.addLine(y=self.set_cam2_y)
        else:
            self.cam2_y_set_line.setValue(self.set_cam2_y)

        # Set the visibility of the lock icon correctly
        if not self.btn_lock.isChecked():  # Then, the program is not locking.
            self.ROICam1_Lock.setVisible(False)
            self.ROICam2_Lock.setVisible(False)
            self.ROICam1_Unlock.setVisible(True)
            self.ROICam2_Unlock.setVisible(True)
        elif self.btn_lock.isChecked():  # Then, the program is locking.
            self.ROICam1_Unlock.setVisible(False)
            self.ROICam2_Unlock.setVisible(False)
            self.ROICam1_Lock.setVisible(True)
            self.ROICam2_Lock.setVisible(True)
        return

    def service_BOSON(self, cam_index):
        """
        Handle the FFCs and NUCs of Boson.
        # TODO: I should operate in manual still and correct this to use the right command.
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

    @pyqtSlot()
    def manual_close(self):
        """
        What to do when app is closing? Release hardware, delete objects, and close threads. Need to write this function
        to not require returning to the GUI Event loop to complete closing. Want everything to happen in this slot!
        """
        # Tell camera threads to quit
        if self.cam1_thread is not None:
            self.cam1_thread.quit()
        if self.cam2_thread is not None:
            self.cam2_thread.quit()

        # Tell the UpdateManager Thread to quit. Nothing else ever tells this thread/object to close. So, they exist!
        self.UpdateManager_thread.quit()
        # Wait on all threads to return.
        if self.cam1_thread is not None:
            self.cam1_thread.wait()
        if self.cam2_thread is not None:
            self.cam2_thread.wait()
        self.UpdateManager_thread.wait()

        # Close Update Manager now, because this function stops and destroys motor threads and destroys motors.
        # BUT Does NOT delete UpdateManager.
        self.UpdateManager.close()

        # All but this (GUI/main) thread are now closed—no event loops running. Therefore, direct calls to QObjects are
        # thread safe, because, only this thread is running, i.e. no race conditions.

        # Close and delete cameras. Delete camera threads:
        if self.cam1 is not None:
            self.cam1._app_closing = True
            self.cam1.close()
            del self.cam1
        if self.cam1_thread is not None:
            del self.cam1_thread
        if self.cam2 is not None:
            self.cam2._app_closing = True
            self.cam2.close()
            del self.cam2
        if self.cam2_thread is not None:
            del self.cam2_thread

        # delete updatemanager
        del self.UpdateManager
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
    def apply_cam1_ROI(self):
        """
        Apply the ROI Bounds on the camera.
        """
        if self.cam1_ROI_set:
            self.cam1.ROI_bounds_set_full_view_signal.emit()
        else:
            ymin, xmin = self.ROICam1_crop.pos()
            height, width = self.ROICam1_crop.size()
            ymax = ymin + height
            xmax = xmin + width
            bounds = [xmin, xmax, ymin, ymax]
            # Both updates the bounds of the ROI and immediately applies them.
            self.cam1.ROI_bounds_set_signal.emit(bounds)
        return

    @pyqtSlot(bool)
    def update_cam_ROI_set(self, roi_is_set: bool, cam_num: int):
        """
        Keeps up with whether an ROI on the camera is set by the user (roi_is_set = TRUE) or if the camera has
        been set to Full View automatically (roi_is_set = False). If the User manually applies a full view ROI, then
        this flag would still be set to True, since user has ser an ROI, eventhough it is full view ROI.
        """
        if cam_num == 1:
            self.cam1_ROI_set = roi_is_set
        elif cam_num == 2:
            self.cam2_ROI_set = roi_is_set
        return

    @pyqtSlot()
    def apply_cam2_ROI(self):
        """
        Apply the ROI Bounds on the camera.
        """
        if self.cam2_ROI_set:
            self.cam2.ROI_bounds_set_full_view_signal.emit()
        else:
            ymin, xmin = self.ROICam2_crop.pos()
            height, width = self.ROICam2_crop.size()
            ymax = ymin + height
            xmax = xmin + width
            bounds = [xmin, xmax, ymin, ymax]
            # Both updates the bounds of the ROI and immediately applies them.
            self.cam2.ROI_bounds_set_signal.emit(bounds)
        return

    @pyqtSlot()
    def update_PID(self):
        """ Update the PID settings with numbers from the GUI """
        if self.UpdateManager.is_PID:
            P = float(self.le_P.text())
            I = float(self.le_Ti.text())
            D = float(self.le_Td.text())
            self.UpdateManager.request_update_pid_settings.emit(P, I, D)
            self.PID = {'P': P, 'Ti': I, 'Td': D}
            # Update the GUI with the numbers from the UpdateManager settings
            self.Update_GUI_PID()
        else:
            print("You are not using a PID Update Manager. Import and use PIDUpdateManager instead!")
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
    def load_state(self, triggered, method='User_selected_file'):
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
            # TODO: Stop using tk.
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
                                         + "Intiially unlocked at " + str(
            self.Unlocked_report[key]["first unlocked at "])
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
    print("main thread is ", win.thread())
    app.aboutToQuit.connect(win.manual_close)
    sys.exit(app.exec())