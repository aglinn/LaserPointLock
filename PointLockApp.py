# Done - TODO: 1 Add statistics logging.
# Done - TODO: 1 Let me FFT the camera COM coordinates to find out which frequencies are generating the noise.
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

# Improve usability
# In Progress -  TODO: 2 stop using global vars and put this code into a fucking window
# TODO: 2 Make the GUI compatible with multiple screen formats.
# TODO: 2 Stupid bug fix: When I reduce the number of points to be plotted for COM and piezzo values, I need to make the
# TODO: 1 Finish overhauling code to allow for operation with IR Cameras.
#  number of points plotted actually smaller.
# TODO: 2 Generally improve the coding.
#

# Improve speed
# TODO: 3 multithread the program. Try speeding up the set and get voltages for the motors
#

# Test new update algorithms
# In Progress - Garrison TODO: 2 Implement PID, instead of just D.
# TODO: 2 try the other algorithm from the paper I found.
#

# TODO: implement update_cam_2_settings functionality similar to update_cam_1_settings.
# TODO: Is there anything else we want to do before launching the StateMachine? Before launch, we can do things like
#   self.StateMachine.updateManager.setpos = Home. On this note, should we only launch state machine from a button so the
#   user has time to fill out all of the settings etc. first, which may be easier functionality to implement without creating
#   states?
# TODO: Would it be better for us to make the MainWindow a public method, which we can then call inside of the state machine?

import sys

import numpy as np
import pyqtgraph as pg
from Packages.pointing_ui import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
import tkinter as tk
from tkinter import filedialog
from Packages.States.QueuedMessageHandler import QueuedMessageHandler, MessageQueue
from typing import List

vector = List[float]


class GuiSignals(QtCore.QObject):
    """Simple class that encapsulates events from GUI functions. Must subclass QObject"""
    report_ready = QtCore.pyqtSignal()


class App:

    def __init__(self):
        # Create Window w/ main loop
        self.app = QtGui.QApplication(sys.argv)
        self.MainWindow = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.MainWindow)

        self.system = "VIS"

        # Define a state machine
        self.StateMachine = QueuedMessageHandler()
        self._StateMachineStatus = 2  # i.e. the state machine is initially not running. 0= idle, 1 = running

        #Instantiate GUI Self-made Signals:
        self.signals = GuiSignals()

        # Define a Message Queue for the GUI. Various signals will enqueue a message into the que, which will be added
        # to the state machine message queue when the state machine stops.
        self.MessageQueue = MessageQueue()
        self.MessageQueue.states = self.StateMachine.message_que.states  #same allowable states in this queue as SM.

        # Instantiate Threadpool:
        self.threadpool = QtCore.QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

        # Instantiate a lock object for thread control
        self.lock = QtCore.QReadWriteLock()

        ## UI Stuff
        # pyqtgraph
        pg.setConfigOptions(imageAxisOrder='row-major')
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        # Create Camera and Motor Dropdowns, error dialog as part of the ui
        self.ui.cam_model = QtGui.QStandardItemModel()
        self.ui.motor_model = QtGui.QStandardItemModel()
        self.ui.error_dialog = QtWidgets.QErrorMessage()
        self.ui.cb_cam1.setModel(self.ui.cam_model)
        self.ui.cb_cam2.setModel(self.ui.cam_model)
        self.ui.cb_motors_1.setModel(self.ui.motor_model)
        self.ui.cb_motors_2.setModel(self.ui.motor_model)

        # Set models and default values for combo boxes
        if len(self.StateMachine.motor_manager.getDeviceList()) >= 2:
            self.ui.cb_motors_1.setCurrentIndex(0)
            self.ui.cb_motors_2.setCurrentIndex(1)
        else:
            self.ui.cb_motors_1.setCurrentIndex(-1)
            self.ui.cb_motors_1.setCurrentIndex(-1)

        self.toggle_mightex_cam_settings_ui_vis(False)
        self.toggle_general_cam_settings_ui_vis(False)
        self.toggle_BOSON_cam_settings_ui_vis(False)

        # Connect UI button to functions
        self.ui.btn_cam1_update.clicked.connect(self.update_cam_1_settings)
        self.ui.btn_cam2_update.clicked.connect(self.update_cam_2_settings)
        self.ui.btn_motor_connect.clicked.connect(self.update_motors)
        self.ui.act_calibrate.triggered.connect(self.beginCalibration)
        self.ui.actionLoad_Old_Home.triggered.connect(self.load_home)
        self.ui.btn_clear.clicked.connect(self.clear_pointing_plots)
        self.ui.btn_lock.clicked.connect(self.lock_pointing)
        self.ui.btn_Home.clicked.connect(self.define_home)
        self.ui.btn_Align.clicked.connect(self.beginAlign)
        self.ui.cb_SystemSelection.currentIndexChanged.connect(self.set_system)  # Dropdown Selector for Vis or IR modes

        # Connect Signals to functions.
        self.StateMachine.signals.SM_done_running.connect(self.StateMachineStopped)
        self.StateMachine.signals.SM_ready_to_report.connect(self.CopyStateMachineReport)
        self.signals.report_ready.connect(self.UpdateGuiWithReport)
        self.StateMachine.signals.SM_Idle.connect(self.StateMachineIdle)

        # Initialize Properties
        self._incoming_report = {}

        self.updateUI()

        #Instantiate a thread lock; so we can

        self.MainWindow.show()

        sys.exit(self.app.exec_())

    def CopyStateMachineReport(self):
        """
        Just copy the StateMachine's report to the App class; so that the SM can get back to work on the next update.
        """
        self._incoming_report = self.StateMachine.report
        self.StateMachine._reporting = False
        self.signals.report_ready.emit()
        return

    def UpdateGuiWithReport(self):
        """
        use self._incoming_report to update the GUI accordingly.
        """
        state = self._incoming_report["state"]
        if state == self.MessageQueue.states["measure"]:
            self.update_cam_1_img(self._incoming_report["cam_1_img"], self._incoming_report["cam_1_reset"])
            self.update_cam_2_img(self._incoming_report["cam_2_img"])
            self.update_cam_1_COM(self._incoming_report["cam_1_com"])
            self.update_cam_2_COM(self._incoming_report["cam_2_com"])
            if not self._incoming_report["dx"] is None:
                self.update_std(self._incoming_report["std"])
        elif state == self.MessageQueue.states["cam_1_settings"]:
            if int(self.ui.cb_SystemSelection.currentIndex()) == 0:
                pass
            elif int(self.ui.cb_SystemSelection.currentIndex()) == 1:
                self.ui.le_cam1_exp_time.setText('%.2f' % (self._incoming_report["exposure_time"]))
                self.ui.le_cam1_gain.setText('%.2f' % (self._incoming_report["gain"]))
            elif int(self.ui.cb_SystemSelection.currentIndex()) == 2:
                pass

            self.resetHist(self.ui.gv_camera1)
        else:
            # Let's not assume every state must report something.
            pass
        return

    def StateMachineStopped(self):
        """
        Should we do anything if the SM stops?
        """
        pass

    def LaunchStateMachine(self):
        """
        Run StateMachine. Run on a new thread.
        """
        if len(self.StateMachine.message_que._state_queue) != 0:
            self._StateMachineStatus = 1  # State Machine is running
            # Launch thread!
            self.threadpool.start(self.StateMachine.run_thread)
        else:
            self._StateMachineStatus = 2  # State Machine is not running!
        return

    def StateMachineIdle(self):
        """
        Probably a worthless function, but I will save it in case we want to do something in this case, but this case
        should never happen.
        """
        if self.StateMachine._reporting:
            return
        else:
            self._StateMachineStatus = 0  # Now the state machine is known to be idle.

            # pass over the Queue from the GUI Message Queue to the SM Queue.
            for i in range(len(self.MessageQueue._state_queue)):
                self.StateMachine.lock.lockForWrite()
                self.StateMachine.message_que.EnqueueMessage(self.MessageQueue.state_queue,
                                                         self.MessageQueue.state_priority,
                                                         self.MessageQueue.state_inputs)
                self.StateMachine.lock.unlock()

            # If the state machine has queued items, get to it state machine!
            if len(self.StateMachine.message_que._state_queue) != 0:
                self._StateMachineStatus = 1  # State Machine should run if queue != 0
        return

    def AttemptToEnqueMessageOnSM(self):
        # pass over the Queue from the GUI Message Queue to the SM Queue.
        for i in range(len(self.MessageQueue._state_queue)):
            self.StateMachine.lock.lockForWrite()
            self.StateMachine.message_que.EnqueueMessage(self.MessageQueue.state_queue,
                                                         self.MessageQueue.state_priority,
                                                         self.MessageQueue.state_inputs)
            self.StateMachine.lock.unlock()

        if self._StateMachineStatus == 2:
            self.LaunchStateMachine()  # Start State Machine as soon as it has a queue.

    def set_system(self):
        """
        Select IR or visible pointlock system, mostly choose cameras to pull in.
        """
        sys_index = int(self.ui.cb_SystemSelection.currentIndex())
        if sys_index == 1:
            self.system = "VIS"
            self.StateMachine.camera_manager.enableMightexAPI()
        elif sys_index == 2:
            self.system = "IR"
        self.updateUI()

    def updateUI(self):
        # Make appropriate Camera Settings available in GUI:
        if self.system == "VIS":
            self.toggle_BOSON_cam_settings_ui_vis(False)
            self.toggle_mightex_cam_settings_ui_vis(True)
            self.toggle_general_cam_settings_ui_vis(True)
        elif self.system == "IR":
            self.toggle_mightex_cam_settings_ui_vis(False)
            self.toggle_general_cam_settings_ui_vis(True)
            self.toggle_BOSON_cam_settings_ui_vis(True)
        else:
            print("Choose a system!")
            return None

        self.update_motor_list()
        self.update_camera_list()

    def update_motor_list(self):
        """
        Add motors to the list of available motors to choose form in the UI.
        """
        self.ui.motor_model.clear()
        for dev in self.StateMachine.motor_manager.getDeviceList():
            # TODO: Is this a bug? Same for update camera. Not sure that '{dev.kind}' calls dev.kind and returns a string
            #  Or does it literally interpret that as a string? Pycharm seems to think dev is unused...
            dropdownItem = QtGui.QStandardItem(f"{dev.kind}:{dev.serial_no}")
            self.ui.motor_model.appendRow(dropdownItem)

    def update_motors(self):
        pass

    def update_camera_list(self):
        """
        Find Cameras and add them to the list of cameras to choose from in the UI.
        """
        self.ui.cam_model.clear()
        self.StateMachine.camera_manager.find_devices()
        print(self.StateMachine.camera_manager.DeviceList)
        # Find the cameras
        for cam in self.StateMachine.camera_manager.DeviceList:
            dropdownItem = QtGui.QStandardItem(f'{cam.kind}:{cam.serial_no}')
            self.ui.cam_model.appendRow(dropdownItem)

    # Set the camera settings buttons invisible, until a Point Lock System is chosen. Then, make visible the correct
    # set of gui buttons.
    def toggle_mightex_cam_settings_ui_vis(self, Logic):
        """
        Set visibility state of Mightex camera settings UI to true or false. i.e. visibility of settings
        relavent to only Mightex cameras.
        input: True/False
        """
        self.ui.label.setVisible(Logic)
        self.ui.label_2.setVisible(Logic)
        self.ui.le_cam1_exp_time.setVisible(Logic)
        self.ui.le_cam1_gain.setVisible(Logic)
        self.ui.cb_cam1_decimate.setVisible(Logic)
        self.ui.label_5.setVisible(Logic)
        self.ui.label_6.setVisible(Logic)
        self.ui.le_cam2_exp_time.setVisible(Logic)
        self.ui.le_cam2_gain.setVisible(Logic)
        self.ui.cb_cam2_decimate.setVisible(Logic)

    def toggle_general_cam_settings_ui_vis(self, Logic):
        """
        Set visibility state of generally useful camera settings UI to true or false. i.e. visibility of settings
        relavent to both Mightex and Boson cameras.
        input: True/False
        """
        self.ui.btn_cam1_update.setVisible(Logic)
        self.ui.btn_cam2_update.setVisible(Logic)
        self.ui.label_7.setVisible(Logic)
        self.ui.label_8.setVisible(Logic)
        self.ui.le_cam1_threshold.setVisible(Logic)
        self.ui.le_cam2_threshold.setVisible(Logic)
        self.ui.label_4.setVisible(Logic)
        self.ui.label_3.setVisible(Logic)
        self.ui.label_7.setVisible(Logic)
        self.ui.cb_cam1.setVisible(Logic)
        self.ui.cb_cam2.setVisible(Logic)
        self.ui.label_14.setVisible(Logic)
        self.ui.label_15.setVisible(Logic)
        self.ui.Cam1_AvgFrames.setVisible(Logic)
        self.ui.Cam2_AvgFrames.setVisible(Logic)
        self.ui.label_9.setVisible(Logic)
        self.ui.label_10.setVisible(Logic)
        self.ui.le_cam1_max.setVisible(Logic)
        self.ui.le_cam2_max.setVisible(Logic)

    def toggle_BOSON_cam_settings_ui_vis(self, Logic):
        """
        Set visibility state of generally useful camera settings UI to true or false. i.e. visibility of settings
        relavent to both Mightex and Boson cameras.
        input: True/False
        """
        self.ui.label_16.setVisible(Logic)
        self.ui.label_17.setVisible(Logic)
        self.ui.label_18.setVisible(Logic)
        self.ui.label_19.setVisible(Logic)

    # Load the Most Recent Home Position:
    def set_home_marker(self):
        global StateMachine
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y = self.StateMachine.update_manager.set_pos
        # pen = pg.mkPen(color=(255, 0, 0), width=2)
        # ROICam1_Unlock = pg.CircleROI(pos=(set_cam1_y-20, set_cam1_x-20), radius=20,
        #                         movable=False, rotatable=False, resizable=False, pen=pen)

        # ROICam2_Unlock = pg.CircleROI(pos=(set_cam2_y-20, set_cam2_x-20), radius=20,
        #                           movable=False, rotatable=False, resizable=False, pen=pen)
        # self.ui.gv_camera1.addItem(ROICam1_Unlock)
        # self.ui.gv_camera2.getView().addItem(ROICam2_Unlock)
        # pen = pg.mkPen(color=(0, 255, 0), width=2)
        # ROICam1_Lock = pg.CircleROI(pos=(set_cam1_y - 20, set_cam1_x - 20), radius=20,
        #                        movable=False, rotatable=False, resizable=False, pen=pen)

        # ROICam2_Lock = pg.CircleROI(pos=(set_cam2_y - 20, set_cam2_x - 20), radius=20,
        #                        movable=False, rotatable=False, resizable=False, pen=pen)
        # self.ui.gv_camera1.addItem(ROICam1_Lock)
        # self.ui.gv_camera2.getView().addItem(ROICam2_Lock)
        state = StateMachine.getState()
        if state == STATE_MEASURE:
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
        elif state == STATE_CALIBRATE:
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(False)
            ROICam2_Unlock.setVisible(False)
        elif state == STATE_LOCKED:
            ROICam1_Unlock.setVisible(False)
            ROICam2_Unlock.setVisible(False)
            ROICam1_Lock.setVisible(True)
            ROICam2_Lock.setVisible(True)
        elif state == STATE_ALIGN:
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)

    # On program init, use this function to load the last used home position automatically
    def load_most_recent_home(self):
        try:
            HomePosition = np.loadtxt('Most_Recent_Home.txt', dtype=float)
            self.StateMachine.update_manager.set_pos = np.asarray(HomePosition)
            self.set_home_marker()
        except OSError:
            self.StateMachine.update_manager.set_pos = None
            print("Hmm there seems to be no saved Home Position, define one before locking.")

    # This opens a file dialog to allow the user to select a different saved home file
    def load_home(self):
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename()
        HomePosition = None
        if self.system == "VIS":
            HomePosition = np.loadtxt(file_path, dtype=float)
            np.savetxt('Most_Recent_Home.txt', HomePosition, fmt='%f')
        elif self.system == "IR":
            HomePosition = np.loadtxt(file_path, dtype=float)
            np.savetxt('Most_Recent_Home_IR.txt', HomePosition, fmt='%f')
        self.StateMachine.update_manager.set_pos = np.asarray(HomePosition)
        print("Set Positions:", HomePosition)
        try:
            ROICam1_Unlock.setVisible(False)
            ROICam2_Unlock.setVisible(False)
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
        except:
            pass
        self.set_home_marker()

    def define_home(self):
        pass

    def shut_down(self):
        self.StateMachine.update_manager.store_data(state=self.StateMachine.getState())
        self.StateMachine.update_manager.reset_data()
        return

    def update_cam_1_img(self, img, reset: bool):
        if reset:
            self.ui.gv_camera1.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False)
        else:
            self.ui.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)

    def update_cam_2_img(self, img):
        pass

    def update_cam_1_COM(self, COM: vector):
        """
        inputs: COM vector of length 2.
        """
        if COM is None:
            # set crosshairs invisible
            pass
        else:
            # ensure visibile crosshairs located at coordinates COM, and update the rolling plots of COM coordinates
            pass

    def update_cam_2_COM(self, COM: vector):
        """
        inputs: COM vector of length 2.
        """
        if COM is None:
            # set crosshairs invisible
            pass
        else:
            # ensure visibile crosshairs located at coordinates COM, and update the rolling plots of COM coordinates
            pass

    def update_cam_1_settings(self):
        cam_1_index = int(self.ui.cb_cam1.currentIndex())
        inputs = {"cam_1_index": cam_1_index}
        if int(self.ui.cb_SystemSelection.currentIndex()) == 0:
            print("Enqueing cam_1_update function from GUI")
            self.MessageQueue.EnqueueMessage(self.MessageQueue.states["cam_1_settings"], 0, inputs=inputs)
        elif int(self.ui.cb_SystemSelection.currentIndex()) == 1:
            # Expect Mightex Cameras
            cam_1_exp_time = float(self.ui.le_cam1_exp_time.text())
            cam_1_gain = float(self.ui.le_cam1_gain.text())
            cam_1_threshold = float(self.ui.le_cam1_threshold.text())
            cam_1_decimate = self.ui.cb_cam1_decimate.isChecked()
            # Define inputs for the state machine.
            inputs.update({"cam_1_exp_time": cam_1_exp_time, "cam_1_gain": cam_1_gain,
                           "cam_1_threshold": cam_1_threshold, "cam_1_decimate": cam_1_decimate})
            self.StateMachine.lock.lockForWrite()
            self.MessageQueue.EnqueueMessage(self.MessageQueue.states["cam_1_settings"], 0, inputs=inputs)
            self.StateMachine.lock.unlock()
        elif int(self.ui.cb_SystemSelection.currentIndex()) == 2:
            # Expect IR Cameras
            pass
        self.AttemptToEnqueMessageOnSM()

    def update_cam_2_settings(self):
        pass

    def update_std(self, std: vector):
        """
        inputs: vector of length 4.
        """
        pass

    def clear_pointing_plots(self):
        pass

    def resetHist(self, histogram_handle):
        pass

    def lock_pointing(self):
        pass

    def beginCalibration(self):
        pass

    def beginAlign(self):
        pass



if __name__ == "__main__":
    app = App()