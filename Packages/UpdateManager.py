import numpy as np
import nfft
import time
from lmfit import Parameters, minimize
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread, QTimer
from Packages.motors import MDT693B_Motor, MDT693A_Motor
from datetime import datetime
import cv2


class update_out_of_bounds(Exception):

    pass


class InsufficientInformation(Exception):

    pass

from matplotlib.pyplot import Figure

class UpdateManager(QObject):
    # The first ints in the signals below indicate which camera/motor 1 or 2, second int is motor_channel

    # Internal Signaling (from myself to myself)
    # cam_img_received_signal = pyqtSignal(int)
    # Internal signal for letting update manager know there is a new COM found post motor update
    # If anything external connects to this signal, I need to redo the lock, align, calibrate transitions.
    com_found_signal = pyqtSignal()
    request_calculate_calibration_matrix_signal = pyqtSignal()

    # emitted by GUI and also sometimes Update Manager to Update Manager
    request_toggle_lock_signal = pyqtSignal(bool)
    request_motors_to_75V_signal = pyqtSignal()

    # emitted by the GUI to the Update Manager signals:
    request_calibrate_signal = pyqtSignal()
    request_update_r0_signal = pyqtSignal(int, np.ndarray)
    request_connect_motor_signal = pyqtSignal(int, str)
    request_set_home_signal = pyqtSignal(np.ndarray)
    request_set_calibration_matrix = pyqtSignal(np.ndarray)
    request_update_num_cameras_connected_signal = pyqtSignal(int) # This int is +1 for adding and -1 for removing
    # bool flags whether to force an unlock or keep trying.
    update_locking_out_of_bounds_params_signal = pyqtSignal(bool, float)
    request_set_camera_threshold_signal = pyqtSignal(int, float)
    request_close = pyqtSignal()
    request_ping = pyqtSignal(float)

    # Signals emitted by Update Manager to the GUI
    update_gui_piezo_voltage_signal = pyqtSignal(int, int, float)
    update_gui_img_signal = pyqtSignal(int, np.ndarray)
    update_gui_locked_state = pyqtSignal(bool)
    update_gui_locking_update_out_of_bounds_signal = pyqtSignal(dict)
    # This reports all COM found regardless of motor status to GUI.
    update_gui_cam_com_signal = pyqtSignal(int, np.ndarray)
    update_gui_new_calibration_matrix_signal = pyqtSignal(np.ndarray)  # send this to GUI for GUI thread to save
    update_gui_ping = pyqtSignal(float)
    request_gui_plot_calibrate_fits = pyqtSignal(Figure)

    # TODO: Implement timing synchronization?
    # TODO: Implement triggering of cameras?
    # TODO: Handle back log of COM captured instead of overwriting, including allowing averaging over multiple images.

    def __init__(self):
        # Init QObject
        super().__init__()
        # These flags become true once com's come in post motors being updated.
        self._cam1_com_updated = False
        self._cam2_com_updated = False
        # Initialize parameters for confirming piezzo updates are applied
        """
        Any time a voltage set is called, the motor and channel number is set false along with motors_updated flag. Once 
        the set returns, the motor number and channel are set true and if all motor/ch flags are true, 
        then the motors_updated is set true. (Failed sets follow logic depending on locking/calibrating to ultimately 
        set these flags true somehow). Once motors_updated is true, then the next acquired com signals connect to either
        apply update or run calibration depending on state. This ensures that pointing information used for update
        or calibrate comes from a img captured, at least partially (want to better synchronize in time), after updated 
        voltages have been applied to the motors.
        """
        self._motors_updated = True  # Flag that is set true once update manager knows all voltages have been updated.
        self.motor1_ch1_updated = True
        self.motor1_ch2_updated = True
        self.motor2_ch1_updated = True
        self.motor2_ch2_updated = True
        # params for locking
        self._locking = False
        self._force_unlock = False
        self.max_time_allowed_unlocked = 120  # s
        self.calibration_matrix = None
        self.set_pos = None
        self.dV = None
        self.update_voltage = None
        self._cam_1_com = np.array([0.0, 0.0])
        self._cam_2_com = np.array([0.0, 0.0])
        self._cam_1_img = None
        self._cam_2_img = None
        self.img1_X_mesh = np.zeros([2, 2])
        self.img1_Y_mesh = np.zeros([2, 2])
        self.img2_X_mesh = np.zeros([2, 2])
        self.img2_Y_mesh = np.zeros([2, 2])
        self._dx = []
        self.V0 = np.array([75.0, 75.0, 75.0, 75.0])
        self._t1 = []
        self._t2 = []
        self._frequency_domain = None
        self._update_GUI_images_every_n_images = 20
        # self._cam1_com_count = self._update_GUI_images_every_n_images
        # self._cam2_com_count = self._update_GUI_images_every_n_images
        self._cam1_img_count = self._update_GUI_images_every_n_images
        self._cam2_img_count = self._update_GUI_images_every_n_images
        self.img1_threshold = 0
        self.img2_threshold = 0
        # Initialize Threading variables:
        self.motor1_thread = None
        self.motor2_thread = None
        self.motor1 = None
        self.motor2 = None
        self.motor1_to_connect = None
        self.motor2_to_connect = None
        self.num_attempts_close_motor1 = 0
        self.num_attempts_close_motor2 = 0
        self.ResourceManager = None  # Only used for old MDT693A piezo controllers.
        self.com_found_signal_handler = None
        # params for tracking unlock status
        self.lock_time_start = None
        self.time_last_unlock = 0
        self.num_out_of_voltage_range = 1
        self.first_unlock = True  # First time since initial lock that piezos went out of bounds?
        self.unlock_report = {}
        self.unlock_counter = 0
        self.time_unlocked_start = 0
        self.time_continually_unlocked = 0
        # parameters for understanding GUI state:
        self.num_cams_connected = 0
        # parameters to handle calibrating:
        self._calibrating = False
        self.num_calibration_points_dropped_current_sweep = 0
        self.num_cams_connected = 0
        self.calibration_pointing_index = None
        self.calibration_sweep_index = None
        self.voltage_step = None
        self.unknown_current_voltage = False
        self.max_setV_attempts_calib = 2
        self.num_attempts_set_motor1_ch1_V = 0
        self.num_attempts_set_motor1_ch2_V = 0
        self.num_attempts_set_motor2_ch1_V = 0
        self.num_attempts_set_motor2_ch2_V = 0
        self.calibration_voltages = 5 * np.arange(31)
        self.num_steps_per_motor = len(self.calibration_voltages)
        self.mot1_x_voltage, self.mot1_y_voltage, self.mot2_x_voltage, self.mot2_y_voltage = np.zeros(
            (4, len(self.calibration_voltages)))
        self.mot1_x_cam1_x, self.mot1_x_cam1_y, self.mot1_x_cam2_x, self.mot1_x_cam2_y = np.zeros(
            (4, len(self.calibration_voltages)))
        self.mot1_y_cam1_x, self.mot1_y_cam1_y, self.mot1_y_cam2_x, self.mot1_y_cam2_y = np.zeros(
            (4, len(self.calibration_voltages)))
        self.mot2_x_cam1_x, self.mot2_x_cam1_y, self.mot2_x_cam2_x, self.mot2_x_cam2_y = np.zeros(
            (4, len(self.calibration_voltages)))
        self.mot2_y_cam1_x, self.mot2_y_cam1_y, self.mot2_y_cam2_x, self.mot2_y_cam2_y = np.zeros(
            (4, len(self.calibration_voltages)))
        self.starting_v = 75.0 * np.ones(4)
        # Start off the calibration process with voltages at first step
        self.starting_v[0] = self.calibration_voltages[0]
        self._r0 = np.array([0, 0, 0, 0])
        self.num_frames_to_average_during_calibrate = 10
        self.cam1_com_avg_num = 1
        self.cam2_com_avg_num = 1
        self._cam1_dx = []
        self._cam2_dx = []
        self.cam1_time_motors_updated = None
        self.cam2_time_motors_updated = None
        self.update_dx = None  # Effective dx to use when finding update voltages.
        self.time_out_interval = 1000.0/55.0  # TODO: Need to set this dynamically.
        self.block_timer = False
        self.is_PID = False
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        return

    def connect_signals(self):
        """
        Connect all update manager emmitted signals to appropriate slots.
        """
        # self.cam_img_received_signal.connect(self.process_img)
        self.request_set_home_signal.connect(lambda v: setattr(self, 'set_pos', v))
        self.request_set_calibration_matrix.connect(lambda mat: setattr(self, 'calibration_matrix', mat))
        self.request_connect_motor_signal.connect(self.connect_motor)
        self.update_locking_out_of_bounds_params_signal.connect(self.update_lock_parameters)
        self.request_toggle_lock_signal.connect(self.lock_pointing)
        self.request_calibrate_signal.connect(self.begin_calibration)
        self.request_update_num_cameras_connected_signal.connect(self.update_num_cameras_connected)
        self.request_calculate_calibration_matrix_signal.connect(self.calculate_calibration_matrix)
        self.request_motors_to_75V_signal.connect(self.motors_to_75V)
        self.request_update_r0_signal.connect(self.update_r0)
        self.request_set_camera_threshold_signal.connect(self.update_img_thresholds)
        self.request_close.connect(self.close)
        self.request_ping.connect(self.return_ping)
        self.timer.timeout.connect(self.apply_update)
        return

    @pyqtSlot(float)
    def return_ping(self, start_time: float):
        self.update_gui_ping.emit(time.monotonic() - start_time)
        return

    # Motor related methods.
    def connect_motor_signals(self, motor_number: int):
        """
        Connect  all motor signals emitted to appropraite update manager slots and emit update gui signals.
        """
        if motor_number == 1:
            self.motor1.connect_signals()
            self.motor1.destroyed.connect(lambda args: self.reconnect_motor(1))
            self.motor1.close_complete_signal.connect(self.accept_motor_close)
            self.motor1.close_fail_signal.connect(self.close_motor_again)
            # ch1 related signals
            # self.motor1.request_set_ch1V_signal.connect(lambda voltage: self.requested_set_motor(1, 1, voltage))
            self.motor1.set_ch1V_complete_signal.connect(self.receive_motor_set_V_complete_signal)
            self.motor1.set_ch1V_complete_signal.connect(lambda motor_num, motor_ch, voltage:
                                                         self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                   voltage))
            self.motor1.set_ch1V_failed_signal.connect(self.receive_motor_set_V_failed_signal)
            self.motor1.get_ch1V_failed_signal.connect(lambda motor_num, motor_ch:
                                                       self.receive_V_failed_signal(motor_num, motor_ch))
            self.motor1.returning_ch1V_signal.connect(self.set_V_from_get)
            self.motor1.returning_ch1V_signal.connect(lambda motor_num, motor_ch, voltage:
                                                      self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                voltage))
            # ch2 related signals
            # self.motor1.request_set_ch2V_signal.connect(lambda voltage: self.requested_set_motor(1, 2, voltage))
            self.motor1.set_ch2V_complete_signal.connect(self.receive_motor_set_V_complete_signal)
            self.motor1.set_ch2V_complete_signal.connect(lambda motor_num, motor_ch, voltage:
                                                         self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                   voltage))
            self.motor1.set_ch2V_failed_signal.connect(self.receive_motor_set_V_failed_signal)
            self.motor1.get_ch2V_failed_signal.connect(lambda motor_num, motor_ch:
                                                       self.receive_V_failed_signal(motor_num, motor_ch))
            self.motor1.returning_ch2V_signal.connect(self.set_V_from_get)
            self.motor1.returning_ch2V_signal.connect(lambda motor_num, motor_ch, voltage:
                                                      self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                voltage))
        elif motor_number == 2:
            self.motor2.connect_signals()
            self.motor2.destroyed.connect(lambda args: self.reconnect_motor(2))
            self.motor2.close_complete_signal.connect(self.accept_motor_close)
            self.motor2.close_fail_signal.connect(self.close_motor_again)
            # ch1 related signals
            # self.motor2.request_set_ch1V_signal.connect(lambda voltage: self.requested_set_motor(2, 1, voltage))
            self.motor2.set_ch1V_complete_signal.connect(self.receive_motor_set_V_complete_signal)
            self.motor2.set_ch1V_complete_signal.connect(lambda motor_num, motor_ch, voltage:
                                                         self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                   voltage))
            self.motor2.set_ch1V_failed_signal.connect(self.receive_motor_set_V_failed_signal)
            self.motor2.get_ch1V_failed_signal.connect(lambda motor_num, motor_ch:
                                                       self.receive_V_failed_signal(motor_num, motor_ch))
            self.motor2.returning_ch1V_signal.connect(self.set_V_from_get)
            self.motor2.returning_ch1V_signal.connect(lambda motor_num, motor_ch, voltage:
                                                      self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                voltage))
            # ch2 related signals
            # self.motor2.request_set_ch2V_signal.connect(lambda voltage: self.requested_set_motor(2, 2, voltage))
            self.motor2.set_ch2V_complete_signal.connect(self.receive_motor_set_V_complete_signal)
            self.motor2.set_ch2V_complete_signal.connect(lambda motor_num, motor_ch, voltage:
                                                         self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                   voltage))
            self.motor2.set_ch2V_failed_signal.connect(self.receive_motor_set_V_failed_signal)
            self.motor2.get_ch2V_failed_signal.connect(lambda motor_num, motor_ch:
                                                       self.receive_V_failed_signal(motor_num, motor_ch))
            self.motor2.returning_ch2V_signal.connect(self.set_V_from_get)
            self.motor2.returning_ch2V_signal.connect(lambda motor_num, motor_ch, voltage:
                                                      self.update_gui_piezo_voltage_signal.emit(motor_num, motor_ch,
                                                                                                voltage))
        # self.request_update_motor_connections_signal.emit(motor_number)
        return

    @pyqtSlot()
    def motors_to_75V(self):
        """
        Set any connected motors back to 75V.
        """
        if self.motor1 is not None:
            self.request_set_motor(1, 1, 75.0)
            self.request_set_motor(1, 2, 75.0)
        if self.motor2 is not None:
            self.request_set_motor(2, 1, 75.0)
            self.request_set_motor(2, 2, 75.0)
        return

    def request_set_motor(self, motor_number: int, motor_chanel: int, voltage: float):
        """
        This function helps keep track of whether motor voltages are not yet set since last set call by setting motor/
        channel false along with motors updated to false, since a set request has been made.
        And how many times a set has been called in a row without a successful set.
        """
        # Because any motor was requested to be set, the motors are not updated, and no camera COM are updated post
        # motor update.
        self._motors_updated = False
        self._cam1_com_updated = False
        self._cam2_com_updated = False
        self.cam1_time_motors_updated = None
        self.cam2_time_motors_updated = None
        self.block_timer = False
        print("update requested.", self.block_timer)
        if self._calibrating:
            self.cam1_com_avg_num = 1
            self.cam2_com_avg_num = 1
        if motor_number == 1:
            if motor_chanel == 1:
                self.motor1_ch1_updated = False
                # print("requesting set 1,1 ", voltage, self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated,
                #      self.motor2_ch2_updated)
                self.num_attempts_set_motor1_ch1_V += 1
                self.motor1.request_set_ch1V_signal.emit(voltage)
            elif motor_chanel == 2:
                self.motor1_ch2_updated = False
                # print("requesting set 1,2 ", voltage, self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated,
                #      self.motor2_ch2_updated)
                self.num_attempts_set_motor1_ch2_V += 1
                self.motor1.request_set_ch2V_signal.emit(voltage)
        elif motor_number == 2:
            if motor_chanel == 1:
                self.motor2_ch1_updated = False
                # print("requesting set 2,1 ", voltage, self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated,
                #       self.motor2_ch2_updated)
                self.num_attempts_set_motor2_ch1_V += 1
                self.motor2.request_set_ch1V_signal.emit(voltage)
            elif motor_chanel == 2:
                self.motor2_ch2_updated = False
                # print("requesting set 2,2 ", voltage, self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated,
                #       self.motor2_ch2_updated)
                self.num_attempts_set_motor2_ch2_V += 1
                self.motor2.request_set_ch2V_signal.emit(voltage)
        return

    @pyqtSlot(int, int)
    def receive_v_failed_signal(self, motor_number: int, motor_ch: int):
        """
        What to do when a get voltage fails??
        When trying to get V0 at the start of locking, just set V0 element to 75.0, which may be a bad assumption, but
            this should only cause a little startup noise
        When trying to get a voltage during calibration... delete the corresponding point in the calibration data.
            Warn the user if I was trying to set a motor to 75.0 V during another motors sweep, since I want to
            calibrate around the central voltages.
        """
        if motor_number == 1:
            if motor_ch == 1:
                if self._locking:
                    self.V0[0] = 75.0
                    self.motor1_ch1_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # failed get—only called when sets fail repeatedly and then a get fails too.
                    self.motor1_ch1_updated = True
                    # Let the calibration run function know that the voltages are unknown so that it drops the next
                    # pointing information captured.
                    self.unknown_current_voltage = True
                    if self.calibration_sweep_index == 0:
                        # Drop this the motor voltage from calibration data.
                        del self.mot1_x_voltage[self.voltage_step - 1 -
                                                self.num_calibration_points_dropped_current_sweep]
                    else:
                        # Well, I really want to calibrate around the center of all of my voltages, and failing here
                        # Means that that will not happen... At least, warn the user with a print statement for now.
                        print("WARNING: During calibration sweeps, motor 1 channel 1 could not be set to 75.0 V for "
                              "another motors sweep.")
            elif motor_ch == 2:
                if self._locking:
                    self.V0[1] = 75.0
                    self.motor1_ch2_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # failed get—only called when sets fail repeatedly and then a get fails too.
                    self.motor1_ch2_updated = True
                    # Let the calibration run function know that the voltages are unknown so that it drops the next
                    # pointing information captured.
                    self.unknown_current_voltage = True
                    if self.calibration_sweep_index == 1:
                        # Drop this the motor voltage from calibration data.
                        del self.mot1_y_voltage[self.voltage_step - 1 -
                                                self.num_calibration_points_dropped_current_sweep]
                    else:
                        # Well, I really want to calibrate around the center of all of my voltages, and failing here
                        # Means that that will not happen... At least, warn the user with a print statement for now.
                        print("WARNING: During calibration sweeps, motor 1 channel 2 could not be set to 75.0 V for "
                              "another motors sweep.")
        elif motor_number == 2:
            if motor_ch == 1:
                if self._locking:
                    self.V0[2] = 75.0
                    self.motor2_ch1_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # failed get—only called when sets fail repeatedly and then a get fails too.
                    self.motor2_ch1_updated = True
                    # Let the calibration run function know that the voltages are unknown so that it drops the next
                    # pointing information captured.
                    self.unknown_current_voltage = True
                    if self.calibration_sweep_index == 2:
                        # Drop this the motor voltage from calibration data.
                        del self.mot2_x_voltage[self.voltage_step - 1 -
                                                self.num_calibration_points_dropped_current_sweep]
                    else:
                        # Well, I really want to calibrate around the center of all of my voltages, and failing here
                        # Means that that will not happen... At least, warn the user with a print statement for now.
                        print("WARNING: During calibration sweeps, motor 2 channel 1 could not be set to 75.0 V for "
                              "another motors sweep.")
            elif motor_ch == 2:
                if self._locking:
                    self.V0[3] = 75.0
                    self.motor2_ch2_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # failed get—only called when sets fail repeatedly and then a get fails too.
                    self.motor2_ch2_updated = True
                    # Let the calibration run function know that the voltages are unknown so that it drops the next
                    # pointing information captured.
                    self.unknown_current_voltage = True
                    if self.calibration_sweep_index == 3:
                        # Drop this the motor voltage from calibration data.
                        del self.mot1_y_voltage[self.voltage_step - 1 -
                                                self.num_calibration_points_dropped_current_sweep]
                    elif self.calibration_sweep_index < 4:
                        # Well, I really want to calibrate around the center of all of my voltages, and failing here
                        # Means that that will not happen... At least, warn the user with a print statement for now.
                        print("WARNING: During calibration sweeps, motor 2 channel 2 could not be set to 75.0 V for "
                              "another motors sweep.")
        self.check_all_motors_updated()
        return

    @pyqtSlot(int, int, float)
    def set_V_from_get(self, motor_number: int, motor_ch: int, voltage: float):
        """
        sets voltages based on a get voltage call.
        if locking, only called at start of locking to update V0 element with starting voltages.
        if calibrating, set sweeping motor voltages accordingly to use when calculating calibration_matrix
        """
        if motor_number == 1:
            if motor_ch == 1:
                if self._locking:
                    self.V0[0] = voltage
                    self.motor1_ch1_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # successful get—only called when sets fail repeatedly.
                    self.motor1_ch1_updated = True
                    if self.calibration_sweep_index == 0:
                        # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                        self.mot1_x_voltage[self.voltage_step - 1 -
                                            self.num_calibration_points_dropped_current_sweep] = voltage
                    else:
                        # I must have failed to set the voltages on this motor to 75 V, right? Warn user.
                        if voltage != 75.0:
                            # Well, I really want to calibrate around the center of all of my voltages, and failing here
                            # Means that that will not happen... At least, warn the user with a print statement for now.
                            print(
                                "WARNING: During calibration sweeps, motor 1 channel 1 could not be set to 75.0 V for "
                                "another motors sweep.")
                        else:
                            print("What? I failed to set voltage to 75.0V, but I am reading 75.0 V somehow??")
            elif motor_ch == 2:
                if self._locking:
                    self.V0[1] = voltage
                    self.motor1_ch2_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # successful get—only called when sets fail repeatedly.
                    self.motor1_ch2_updated = True
                    if self.calibration_sweep_index == 1:
                        # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                        self.mot1_y_voltage[self.voltage_step - 1 -
                                            self.num_calibration_points_dropped_current_sweep] = voltage
                    else:
                        # I must have failed to set the voltages on this motor to 75 V, right? Warn user.
                        if voltage != 75.0:
                            # Well, I really want to calibrate around the center of all of my voltages, and failing here
                            # Means that that will not happen... At least, warn the user with a print statement for now.
                            print(
                                "WARNING: During calibration sweeps, motor 1 channel 2 could not be set to 75.0 V for "
                                "another motors sweep.")
                        else:
                            print("What? I failed to set voltage to 75.0V, but I am reading 75.0 V somehow??")
        elif motor_number == 2:
            if motor_ch == 1:
                if self._locking:
                    self.V0[2] = voltage
                    self.motor2_ch1_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # successful get—only called when sets fail repeatedly.
                    self.motor2_ch1_updated = True
                    if self.calibration_sweep_index == 2:
                        # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                        self.mot2_x_voltage[self.voltage_step - 1 -
                                            self.num_calibration_points_dropped_current_sweep] = voltage
                    else:
                        # I must have failed to set the voltages on this motor to 75 V, right? Warn user.
                        if voltage != 75.0:
                            # Well, I really want to calibrate around the center of all of my voltages, and failing here
                            # Means that that will not happen... At least, warn the user with a print statement for now.
                            print(
                                "WARNING: During calibration sweeps, motor 2 channel 1 could not be set to 75.0 V for "
                                "another motors sweep.")
                        else:
                            print("What? I failed to set voltage to 75.0V, but I am reading 75.0 V somehow??")
            elif motor_ch == 2:
                if self._locking:
                    self.V0[3] = voltage
                    self.motor2_ch2_updated = True
                elif self._calibrating:
                    # Failed sets do not reset the motor updated flag in calibration state. So, reset the flag from a
                    # successful get—only called when sets fail repeatedly.
                    self.motor2_ch2_updated = True
                    if self.calibration_sweep_index == 3:
                        # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                        self.mot2_y_voltage[self.voltage_step - 1 -
                                            self.num_calibration_points_dropped_current_sweep] = voltage
                    else:
                        # I must have failed to set the voltages on this motor to 75 V, right? Warn user.
                        if voltage != 75.0:
                            # Well, I really want to calibrate around the center of all of my voltages, and failing here
                            # Means that that will not happen... At least, warn the user with a print statement for now.
                            print(
                                "WARNING: During calibration sweeps, motor 2 channel 2 could not be set to 75.0 V for "
                                "another motors sweep.")
                        else:
                            print("What? I failed to set voltage to 75.0V, but I am reading 75.0 V somehow??")
        self.check_all_motors_updated()
        return

    @pyqtSlot(int)
    def accept_motor_close(self, motor_number: int):
        """
        Motor has closed. So, reset number of attempts at closing.
        """
        if motor_number == 1:
            self.num_attempts_close_motor1 = 0
        elif motor_number == 2:
            self.num_attempts_close_motor2 = 0
        return

    @pyqtSlot(int)
    def accept_motor_thread_close(self, motor_thread_num: int):
        if motor_thread_num == 1:
            # Event loop has closed. So, now I can directly call slots of the motors safely.
            # Call close method of motor: closes the motor and also deletes itself.
            self.motor1.close()
            # Now, delete the thread.
            del self.motor1_thread
        elif motor_thread_num == 2:
            # Event loop has closed. So, now I can directly call slots of the motors safely.
            # Call close method of motor: closes the motor and also deletes itself.
            self.motor2.close()
            # Now, delete the thread.
            del self.motor2_thread
        # Now that the motors and threads are closed, I can delete myself.
        self.deleteLater()
        print('Closed motor thread num: ', motor_thread_num)
        return

    def __del__(self):
        print("Update Manager Destructor called.")
        return

    @pyqtSlot(int)
    def close_motor_again(self, motor_number: int):
        """
        Try to close motor again. Only try 3 times total.
        """
        if motor_number == 1:
            if self.num_attempts_close_motor1 < 3:
                self.motor1.request_close_signal.emit()
                self.num_attempts_close_motor1 += 1
        elif motor_number == 2:
            if self.num_attempts_close_motor2 < 3:
                self.motor2.request_close_signal.emit()
                self.num_attempts_close_motor2 += 1
        return

    @pyqtSlot(int, str)
    def connect_motor(self, motor_number: int, motor_to_connect: str):
        """
        This function connects motor_number as motor_to_connect.
        If first time connecting motor_number, generate thread for motor and connect.
        If not first time, then delete old motor and eventually connect new motor (after signaling around).
        """
        if motor_number == 1:
            if self.motor1_thread is None:  # First time that a motor is connected as motor 1!
                # Start a thread for the motor to live in
                self.motor1_thread = QThread()
                self.motor1_thread.started.connect(lambda: print("Motor 1 thread has started."))
                self.motor1_thread.finished.connect(lambda: print("Motor 1 thread has finished."))
                # Connect motor_1_thread.finished signal to a method to clean up when closing.
                self.motor1_thread.finished.connect(lambda: self.accept_motor_thread_close(1))
                # Instantiate a motor
                if "MDT693B" in motor_to_connect:
                    self.motor1 = MDT693B_Motor(str(motor_to_connect[2:15]), motor_number=1)
                else:
                    if self.ResourceManager is None:
                        import visa
                        self.ResourceManager = visa.ResourceManager()
                    self.motor1 = MDT693A_Motor(self.ResourceManager, motor_number=1, com_port=motor_to_connect,
                                                ch1='X',
                                                ch2='Y')
                # Move the motor to its thread
                self.motor1.moveToThread(self.motor1_thread)
                # Connect all signals from the motors
                self.connect_motor_signals(1)
                # Start the thread
                # See priority options here: https://doc.qt.io/qt-6/qthread.html#Priority-enum
                # Set priority as time critical! As soon as I want to apply an update, tell the motors and apply ASAP!
                self.motor1_thread.start(priority=6)
            else:  # Want to swap motors out on thread 1!
                # Request that we close motor 1 first.
                self.motor1.request_close_signal.emit()
                # increment number of close attempts
                self.num_attempts_close_motor1 = 1
                self.motor1_to_connect = motor_to_connect
        if motor_number == 2:
            if self.motor2_thread is None:  # First time that a motor is connected as motor 1!
                # Start a thread for the motor to live in
                self.motor2_thread = QThread()
                self.motor2_thread.started.connect(lambda: print("Motor 2 thread has started."))
                self.motor2_thread.finished.connect(lambda: print("Motor 2 thread has finished."))
                # Connect motor_1_thread.finished signal to a method to clean up when closing.
                self.motor2_thread.finished.connect(lambda: self.accept_motor_thread_close(2))
                # Instantiate a motor
                if "MDT693B" in motor_to_connect:
                    self.motor2 = MDT693B_Motor(str(motor_to_connect[2:15]), motor_number=2)
                else:
                    if self.ResourceManager is None:
                        import visa
                        self.ResourceManager = visa.ResourceManager()
                    self.motor2 = MDT693A_Motor(self.ResourceManager, motor_number=2, com_port=motor_to_connect,
                                                ch1='X', ch2='Y')
                # Move the motor to its thread
                self.motor2.moveToThread(self.motor2_thread)
                # Connect all signals from the motors
                self.connect_motor_signals(2)
                # Start the thread
                # See priority options here: https://doc.qt.io/qt-6/qthread.html#Priority-enum
                # Set priority as time critical! As soon as I want to apply an update, tell the motors and apply ASAP!
                self.motor2_thread.start(priority=6)
            else:  # Want to swap motors out on thread 1!
                # Request that we close motor 1 first.
                self.motor2.request_close_signal.emit()
                # increment number of close attempts
                self.num_attempts_close_motor2 = 1
                self.motor2_to_connect = motor_to_connect
        return

    @pyqtSlot(int)
    def reconnect_motor(self, motor_number: int):
        """
        Called when an old motor object is deleted. It connects a new motor as self.motor_to_connect.
        """
        if motor_number == 1:
            if self.motor1_to_connect is not None:
                # Instantiate a motor
                if "MDT693B" in self.motor1_to_connect:
                    self.motor1 = MDT693B_Motor(str(self.motor1_to_connect[2:15]), motor_number=1)
                else:
                    if self.ResourceManager is None:
                        self.ResourceManager = visa.ResourceManager()
                    self.motor1 = MDT693A_Motor(self.ResourceManager, motor_number=1, com_port=self.motor1_to_connect, ch1='X', ch2='Y')
                # Move the motor to its thread
                self.motor1.moveToThread(self.motor1_thread)
                # Connect all signals from the motors
                self.connect_motor_signals(1)
                # Reset motor1_to_connect to None, because it is being connected now.
                self.motor1_to_connect = None
        elif motor_number == 2:
            if self.motor2_to_connect is not None:
                # Instantiate a motor
                if "MDT693B" in self.motor2_to_connect:
                    self.motor2 = MDT693B_Motor(str(self.motor2_to_connect[2:15]), motor_number=2)
                else:
                    if self.ResourceManager is None:
                        self.ResourceManager = visa.ResourceManager()
                    self.motor2 = MDT693A_Motor(self.ResourceManager, motor_number=2, com_port=self.motor2_to_connect,
                                                ch1='X', ch2='Y')
                # Move the motor to its thread
                self.motor2.moveToThread(self.motor2_thread)
                # Connect all signals from the motors
                self.connect_motor_signals(2)
                # Reset motor2_to_connect to None, because it is being connected now.
                self.motor2_to_connect = None
        return

    @pyqtSlot(int, int, float)
    def receive_motor_set_V_complete_signal(self, motor_number: int, motor_ch: int, set_V: float):
        """
        This function handles the signal from the motor that a voltage has been set correctly.
        There are two purposes:
        1) Keeping up with whether the motors have been set since their last set call.
        2) Storing the voltage that was set: 1) when locking as self.V0, and 2) when calibrating, as the appropriate
            motor voltage that was set during calibration.
        """
        # Because this function is only called when a motor set is successful, there should always be at least 1 false
        # flag, so set them true first, then check if all are true.
        if motor_number == 1:
            if motor_ch == 1:
                self.num_attempts_set_motor1_ch1_V = 0
                # print("received set 1,1, correct signal. ", self.motor1_ch1_updated, self.motor1_ch2_updated,
                #      self.motor2_ch1_updated, self.motor2_ch2_updated)
                if self._calibrating and self.calibration_sweep_index == 0:
                    # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                    self.mot1_x_voltage[self.voltage_step - 1 -
                                        self.num_calibration_points_dropped_current_sweep] = set_V
                elif self._locking:
                    self.V0[0] = set_V
                self.motor1_ch1_updated = True
            elif motor_ch == 2:
                self.num_attempts_set_motor1_ch2_V = 0
                # print("received set 1,2 correct signal. ", self.motor1_ch1_updated, self.motor1_ch2_updated,
                #       self.motor2_ch1_updated, self.motor2_ch2_updated)
                if self._calibrating and self.calibration_sweep_index == 1:
                    # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                    self.mot1_y_voltage[self.voltage_step - 1 -
                                        self.num_calibration_points_dropped_current_sweep] = set_V
                elif self._locking:
                    self.V0[1] = set_V
                self.motor1_ch2_updated = True
        elif motor_number == 2:
            if motor_ch == 1:
                self.num_attempts_set_motor2_ch1_V = 0
                # print("received set 2,1 correct signal. ", self.motor1_ch1_updated, self.motor1_ch2_updated,
                #       self.motor2_ch1_updated, self.motor2_ch2_updated)
                if self._calibrating and self.calibration_sweep_index == 2:
                    # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                    self.mot2_x_voltage[self.voltage_step - 1 -
                                        self.num_calibration_points_dropped_current_sweep] = set_V
                elif self._locking:
                    self.V0[2] = set_V
                self.motor2_ch1_updated = True
            elif motor_ch == 2:
                self.num_attempts_set_motor2_ch2_V = 0
                # print("received set 2,2 correct signal. ", self.motor1_ch1_updated, self.motor1_ch2_updated,
                #       self.motor2_ch1_updated, self.motor2_ch2_updated)
                if self._calibrating and self.calibration_sweep_index == 3:
                    # Only set the voltages for the motor that I am sweeping, i.e. ignore reset voltages to 75.0.
                    self.mot2_y_voltage[self.voltage_step - 1 -
                                        self.num_calibration_points_dropped_current_sweep] = set_V
                elif self._locking:
                    self.V0[3] = set_V
                self.motor2_ch2_updated = True

        self.check_all_motors_updated()
        return

    def check_all_motors_updated(self):
        """
        check if all motors have been updated and set flags appropriately.
        """
        if self.motor1_ch1_updated and self.motor1_ch2_updated and self.motor2_ch1_updated and self.motor2_ch2_updated:
            # Yay! All motors have been updated!
            # Set motors updated to true, which tells com update functions to accept next com update as happening after
            # voltages have been set
            self._motors_updated = True
        return

    @pyqtSlot(int, int)
    def receive_motor_set_V_failed_signal(self, motor_number: int, motor_ch: int):
        """
        What to do when a motor voltage set fails??
        When calibrating, try again self.max_setV_attempts_calib times. If that does not work, then get current voltage.
        When locked, simply set the corresponding motor updated flag true, assuming that the voltage for that motor
            and chanel is not updated, thus is what it was previously set to as V0.
        """
        if motor_number == 1:
            if motor_ch == 1:
                if self._calibrating:
                    if self.num_attempts_set_motor1_ch1_V <= self.max_setV_attempts_calib:
                        if self.calibration_sweep_index == 0:
                            # I am trying to set a particular voltage on this motor.
                            self.request_set_motor(1, 1, self.calibration_voltages[self.voltage_step - 1])
                        else:
                            # I should only be trying to reset to 75.0 V
                            self.request_set_motor(1, 1, 75.0)
                    else:
                        # Could not set the voltage correctly. Can I at least get the current voltage?
                        self.motor1.request_get_ch1V_signal.emit()
                        # Reset this flag because will not call set again until a totally new attempt is made.
                        self.num_attempts_set_motor1_ch1_V = 0
                elif self._locking:
                    # Just accept, because presumably, this voltage is unchanged and thus already in V0
                    self.motor1_ch1_updated = True
                    # print("Accepted set 1,1 failed, ", self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated, self.motor2_ch2_updated)
            elif motor_ch == 2:
                if self._calibrating:
                    if self.num_attempts_set_motor1_ch2_V <= self.max_setV_attempts_calib:
                        if self.calibration_sweep_index == 1:
                            # I am trying to set a particular voltage on this motor.
                            self.request_set_motor(1, 2, self.calibration_voltages[self.voltage_step - 1])
                        else:
                            # I should only be trying to reset to 75.0 V
                            self.request_set_motor(1, 2, 75.0)
                    else:
                        # Could not set the voltage correctly. Can I at least get the current voltage?
                        self.motor1.request_get_ch2V_signal.emit()
                        # Reset this flag because will not call set again until a totally new attempt is made.
                        self.num_attempts_set_motor1_ch2_V = 0
                elif self._locking:
                    # Just accept, because presumably, this voltage is unchanged and thus already in V0
                    self.motor1_ch2_updated = True
                    # print("Accepted set 1,2 failed, ", self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated, self.motor2_ch2_updated)
        elif motor_number == 2:
            if motor_ch == 1:
                if self._calibrating:
                    if self.num_attempts_set_motor2_ch1_V <= self.max_setV_attempts_calib:
                        if self.calibration_sweep_index == 2:
                            # I am trying to set a particular voltage on this motor.
                            self.request_set_motor(2, 1, self.calibration_voltages[self.voltage_step - 1])
                        else:
                            # I should only be trying to reset to 75.0 V
                            self.request_set_motor(2, 1, 75.0)
                    else:
                        # Could not set the voltage correctly. Can I at least get the current voltage?
                        self.motor2.request_get_ch2V_signal.emit()
                        # Reset this flag because will not call set again until a totally new attempt is made.
                        self.num_attempts_set_motor2_ch1_V = 0
                elif self._locking:
                    # Just accept, because presumably, this voltage is unchanged and thus already in V0
                    self.motor2_ch1_updated = True
                    # print("Accepted set 2,1 failed, ", self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated, self.motor2_ch2_updated)
            elif motor_ch == 2:
                if self._calibrating:
                    if self.num_attempts_set_motor2_ch2_V <= self.max_setV_attempts_calib:
                        if self.calibration_sweep_index == 3:
                            # I am trying to set a particular voltage on this motor.
                            self.request_set_motor(2, 2, self.calibration_voltages[self.voltage_step - 1])
                        else:
                            # I should only be trying to reset to 75.0 V
                            self.request_set_motor(2, 2, 75.0)
                    else:
                        # Could not set the voltage correctly. Can I at least get the current voltage?
                        self.motor2.request_get_ch2V_signal.emit()
                        # Reset this flag because will not call set again until a totally new attempt is made.
                        self.num_attempts_set_motor2_ch2_V = 0
                elif self._locking:
                    # Just accept, because presumably, this voltage is unchanged and thus already in V0
                    self.motor2_ch2_updated = True
                    # print("Accepted set 2,2 failed, ", self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated, self.motor2_ch2_updated)
        # Only get to this check if I am locking.
        self.check_all_motors_updated()
        return

    # Locking related methods
    @pyqtSlot()
    def time_delay_update(self):
        # print("Calling Update")
        if self._cam1_com_updated and self._cam2_com_updated and not self.block_timer:
            self.block_timer = True
            self.timer.start(self.time_out_interval)
        return

    @pyqtSlot()
    def apply_update(self):
        """
        This function is called by the timer timing out, which is only started when no motors are currently being
        changed, and applies an update if both cam com's have been found post voltage update.
        """
        try:
            # Try getting the update voltage
            self.get_update()
        except update_out_of_bounds:
            """"
            This section of code keeps the update voltage in bounds by finding a voltage that minizes dx while in
            staying in bounds.
            Additionally, it tracks the number of times that the voltages went out of bounds in less than 1 minute 
            since the last out of bounds (i.e. if it goes out of bounds once but then comes back in bounds for more 
            than 1 minute, the counter is reset to 0). If the piezos go out of bounds more than 10 times in a row 
            in less than a minute each time, then the code unlocks and returns to measuring state.
            """
            # Fit the best update we can, given we cannot restore pointing
            self.fit_update()
            # Track unlocking.
            self.report_unlock()
            # Check if I am allowed to continue attempting to lock
            if self._force_unlock:
                if self.time_continually_unlocked > self.max_time_allowed_unlocked:
                    self.num_out_of_voltage_range = 1
                    self.time_last_unlock = 0
                    # Set all voltages back to 75V
                    self.request_motors_to_75V_signal.emit()
                    successful_lock_time = time.monotonic() - self.lock_time_start
                    print("Successfully locked pointing for", successful_lock_time)
                    # Tell myself to stop locking.
                    self._locking = False
                    self.request_toggle_lock_signal.emit(False)
                    print("System has unlocked!")
                    return

        # Apply all update voltages
        self.request_set_motor(1, 1, self.update_voltage[0])
        self.request_set_motor(2, 1, self.update_voltage[2])
        self.request_set_motor(1, 2, self.update_voltage[1])
        self.request_set_motor(2, 2, self.update_voltage[3])
        return

    def report_unlock(self):
        """
        called when an attempted update is out of bounds.
        tracks the number of times that the voltages went out of bounds in less than 1 minute
        since the last out of bounds (i.e. if it goes out of bounds once but then comes back in bounds for more
        than 1 minute, the counter is reset to 0).
        reports unlocked stats to the GUI.
        """
        # Track time since last unlock and the number of times unlocked in less than 1 minute since previous
        # lock.
        if time.monotonic() - self.time_last_unlock < 60.0 and not self.first_unlock:
            self.num_out_of_voltage_range += 1
            self.time_last_unlock = time.monotonic()
            self.unlock_report["round " + str(self.unlock_counter) +
                            " of going out of piezzo range."]["number of successive out of bounds"
                                                              " updates with less than 60 seconds "
                                                              "between updates"] = self.num_out_of_voltage_range

            self.time_continually_unlocked = time.monotonic() - self.time_unlocked_start
            self.unlock_report["round " + str(self.unlock_counter) +
                            " of going out of piezzo range."]["Amount of time spent outside voltage "
                                                              "range"] = self.time_continually_unlocked
            if np.linalg.norm(self.dx[-1]) > self.unlock_report["round " + str(self.unlock_counter) +
                                                                      " of going out of piezzo range."][
                "Farthest dx during this round of unlock"]:
                self.unlock_report["round " + str(self.unlock_counter) +
                                " of going out of piezzo range."][
                    "Farthest dx during this round of unlock"] = np.linalg.norm(self.dx[-1])
        else:
            self.unlock_report = {}  # I will be passing each round rather than all unlock reports at once.
            self.first_unlock = False
            self.time_last_unlock = time.monotonic()  # Used to keep track of how often I am unlocking.
            num_out_of_voltage_range = 1
            self.time_unlocked_start = self.time_last_unlock  # Used to know when I first started the seris of unlock
            self.time_continually_unlocked = 0  # tracks the duration of this series of unlock
            self.unlock_counter += 1
            self.unlock_report["round " + str(self.unlock_counter) +
                            " of going out of piezzo range."] = {"number of successive out of bounds"
                                                                 " updates with less than 60 seconds "
                                                                 "between updates": num_out_of_voltage_range,
                                                                 "Amount of time spent outside voltage "
                                                                 "range": self.time_continually_unlocked,
                                                                 "Farthest dx during this round of unlock":
                                                                     np.linalg.norm(self.dx[-1]),
                                                                 "first unlocked at ": datetime.now()}
        self.update_gui_locking_update_out_of_bounds_signal.emit(self.unlock_report)
        return

    @pyqtSlot(bool)
    def lock_pointing(self, lock: bool):
        """
        Prepare the update manager to begin locking the pointing. And also tell the update manager to prepare to stop
        locking as well.
        lock is True when requesting begin lock and untrue when requesting leave lock.
        """
        # TODO: For PID need to reset self.integral_ti
        if lock:
            # First check that we are ready to lock.
            if self.calibration_matrix is None:
                self.update_gui_locked_state.emit(False)
                raise InsufficientInformation("Run calibration/load calibration before locking")
            if self.motor1 is None:
                self.update_gui_locked_state.emit(False)
                raise InsufficientInformation("Connect motor 1 before locking")
            if self.motor2 is None:
                self.update_gui_locked_state.emit(False)
                raise InsufficientInformation("Connect motor 2 before locking")
            if self.set_pos is None:
                self.update_gui_locked_state.emit(False)
                raise InsufficientInformation("Set/load home position before locking")
            if self.num_cams_connected != 2:
                self.update_gui_locked_state.emit(False)
                raise InsufficientInformation("Need to have 2 cameras connected before, please.")
            # Make sure no handler is connected to com_found_signal
            if self.com_found_signal_handler is not None:
                # Disconnect unwanted connections.
                if self.com_found_signal_handler == 'time_delay_update':
                    # Already connected to the right handler
                    pass
                else:
                    self.com_found_signal.disconnect() # Disconnects from all slots! So far only using 1 slot.
                    # when locking, the COM found signal should trigger the time delay update function.
                    self.com_found_signal.connect(self.time_delay_update)
                    self.com_found_signal_handler = 'time_delay_update'
            else:
                # when locking, the COM found signal should trigger the time delay update function.
                self.com_found_signal.connect(self.time_delay_update)
                self.com_found_signal_handler = 'time_delay_update'
            self._locking = True
            # Get starting voltage for V0, manually set motor updated flags to false and reset to true when gets
            # complete. This ensures I know starting V0 before an attempted update is made.
            # And only sets automatically set these flags false, not the gets used below.
            self.motor1_ch1_updated = False
            self.motor1_ch2_updated = False
            self.motor2_ch1_updated = False
            self.motor2_ch2_updated = False
            self._motors_updated = False
            self.motor1.request_get_ch1V_signal.emit()
            self.motor2.request_get_ch1V_signal.emit()
            self.motor1.request_get_ch2V_signal.emit()
            self.motor2.request_get_ch2V_signal.emit()
            # Initiatlize parameters for tracking unlocking:
            self.lock_time_start = time.monotonic()
            self.time_last_unlock = 0
            self.num_out_of_voltage_range = 1
            self.first_unlock = True  # First time since initial lock that piezos went out of bounds?
            self.update_gui_locked_state.emit(True)
        else:
            if self.com_found_signal_handler is not None:
                # Disconnect unwanted connections.
                self.com_found_signal.disconnect()
                self.com_found_signal_handler = None
            self._locking = False
            self.update_gui_locked_state.emit(False)
        pass

    @pyqtSlot(bool, float)
    def update_lock_parameters(self, force_unlock: bool, max_time_allowed_unlocked: float):
        """
        Communicate any information about how to lock from the GUI. For now, just whether to force an unlock when out
        of bounds or allow continued attempts to lock.
        """
        self._force_unlock = force_unlock
        self.max_time_allowed_unlocked = max_time_allowed_unlocked
        return

    def calc_update_dx(self):
        """
        Find the effective dx to use when calculating the update voltages. In this case just average all values since
        last motor update.
        """
        inds_post_motor_update = np.where(self.t1 >= self.cam1_time_motors_updated)
        dx_cam1_avg = np.average(self.cam1_dx[inds_post_motor_update], axis=0)
        inds_post_motor_update = np.where(self.t2 >= self.cam2_time_motors_updated)
        dx_cam2_avg = np.average(self.cam2_dx[inds_post_motor_update], axis=0)
        self.update_dx = np.concatenate((dx_cam1_avg, dx_cam2_avg), axis=0)
        return

    def get_update(self):
        """
        This code finds the next voltages to apply to the piezo.
        """
        # The convention with dx is how much has the position on the camera changed from the set point.
        self.calc_update_dx()

        # Because of our convention for dX, the meaning of dV is how much would the voltages have changed to result
        # in the change observed in dX
        dV = np.matmul(self.calibration_matrix, self.update_dx)
        self.dV = np.round(dV, decimals=1)  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.
        # voltage to be set on the motors
        # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
        # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
        # so we subtract that dV restoring us to the old position.
        print("Shape of V0 and dV, respectively", self.V0.shape, self.dV.shape)
        update_voltage = self.V0 - self.dV
        print("shape of update voltage, ", update_voltage.shape)
        if np.any(update_voltage > 150) or np.any(update_voltage < 0):
            exception_message = ''
            for i in range(4):
                if update_voltage[i] < 0:
                    exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) +' '
                if update_voltage[i] > 150:
                    exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) +' '
            raise update_out_of_bounds(exception_message)
        else:
            self.update_voltage = update_voltage
            return

    def fit_update(self):
        """
        When the update voltages are out of their allowed range, I want to try to find an update that minimizes
        my dx in the future step, constrained by the allowed piezo ranges. That is what this function does.
        """
        P = Parameters()
        P.add('dv_0', 0, min = -self.V0[0], max = 150.0 - self.V0[0])
        P.add('dv_1', 0, min=-self.V0[1], max=150.0 - self.V0[1])
        P.add('dv_2', 0, min=-self.V0[2], max=150.0 - self.V0[2])
        P.add('dv_3', 0, min=-self.V0[3], max=150.0 - self.V0[3])
        res = minimize(self.residual, P, args=(self.update_dx, self.inv_calibration_matrix))
        dV = np.array([res.params['dv_0'].value, res.params['dv_1'].value, res.params['dv_2'].value,
                            res.params['dv_3'].value])
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.

        # There is a plus sign here instead of a minus, because I am finding and applying a change in voltage that
        # hopefully induces a dx to undo the measured dx, up to constraints on voltages.
        self.update_voltage = self.V0 + self.dV
        return

    @property
    def inv_calibration_matrix(self):
        """
        Return the inverse calibration matrix, which is needed in the fit_update method.
        """
        return np.linalg.inv(self.calibration_matrix)

    @staticmethod
    def residual(params, dx, inv_calibration_matrix):
        """
        This returns the difference between the current measured dx and the hypothetical dx induced by a voltage
        change to the piezzos.
        """
        dV = np.array([params['dv_0'].value, params['dv_1'].value, params['dv_2'].value, params['dv_3'].value])
        dx_induced = np.matmul(inv_calibration_matrix, dV)
        return np.square(dx+dx_induced)

    # Calibration related methods.
    @pyqtSlot()
    def begin_calibration(self):
        """
        This function will transition into the calibration process for update manager.
        """
        # Check if ready to calibrate
        if self.motor1 is None:
            raise InsufficientInformation("Connect motor 1 before calibrating")
        if self.motor2 is None:
            raise InsufficientInformation("Connect motor 2 before calibrating")
        if self.num_cams_connected != 2:
            raise InsufficientInformation("Need to have 2 cameras connected before calibrating, please")
        # Make sure no handler is connected to com_found_signal
        if self.com_found_signal_handler is not None:
            # Disconnect unwanted connections.
            if self.com_found_signal_handler == 'run_calibration':
                # Already connected to the right handler
                pass
            else:
                self.com_found_signal.disconnect()  # Disconnects from all slots! So far only using 1 slot.
                self.com_found_signal.connect(self.run_calibration)
                self.com_found_signal_handler = 'run_calibration'
        else:
            self.com_found_signal.connect(self.run_calibration)
            self.com_found_signal_handler = 'run_calibration'
        self._locking = False
        self.calibration_pointing_index = 0
        self.calibration_sweep_index = 0
        self.voltage_step = 1  # Start at 1, because index 0 is set on motor1 ch1 as starting voltage
        self._calibrating = True
        self.request_set_motor(1, 1, self.starting_v[0])
        self.request_set_motor(2, 1, self.starting_v[2])
        self.request_set_motor(1, 2, self.starting_v[1])
        self.request_set_motor(2, 2, self.starting_v[3])
        return

    @pyqtSlot()
    def run_calibration(self):
        """
        Receieves the COM update signal. Once both are updated, add them to pointing array for later use. Apply next
        voltage in calibration series. Once all voltages are swept through, request to calculate the calibration_matrix.
        """
        print("Called run calibration.", self.calibration_sweep_index, self.calibration_pointing_index)
        if self._cam1_com_updated and self._cam2_com_updated:
            # I have updated COM coordinates to store.
            # Keep updating motors and storing positions
            if self.calibration_sweep_index == 0:
                # capture pointing from first motor, channel 1 sweep
                # Now capture the pointing information from the last voltage step.
                # There won't be a com from the next voltage step yet eventhough called, because imgs can only be
                # received by an event, and the event loop is not being processed during this blocking function.
                if not self.unknown_current_voltage:
                    # Only store captured pointing information, if I know what voltages are on the motors right now.
                    self.mot1_x_cam1_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[0]
                    self.mot1_x_cam1_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[1]
                    self.mot1_x_cam2_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[0]
                    self.mot1_x_cam2_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[1]
                else:  # Whoops never set nor get the voltage for this step, so delete this steps pointing info
                    # Reset unknown voltage flag
                    self.unknown_current_voltage = False
                    del self.mot1_x_cam1_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot1_x_cam1_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot1_x_cam2_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot1_x_cam2_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    self.num_calibration_points_dropped_current_sweep += 1
                # Keep moving calibration pointing index appropriately
                if self.calibration_pointing_index == self.num_steps_per_motor-1:
                    # Finished getting pointing information from this motor sweep
                    self.calibration_pointing_index = 0
                    # Reset number of dropped calibration points
                    self.num_calibration_points_dropped_current_sweep = 0
                    # step to next step sweep calibration
                    self.calibration_sweep_index += 1
                else:
                    # Get next step in pointing information for this sweep
                    self.calibration_pointing_index += 1
                # Go ahead and get the motor to start updating to the next set point
                if self.voltage_step < self.num_steps_per_motor:
                    # Set the next voltage step on motor current motor
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    self.voltage_step += 1
                    self.request_set_motor(1, 1, set_voltage)
                elif self.voltage_step == self.num_steps_per_motor:
                    # Finished voltage steps on the last motor, now start the next motor moving.
                    # But I still need pointing info from the last motor set position.
                    self.voltage_step = 0
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    # Set motor 1 ch1 back to center voltage
                    self.voltage_step += 1
                    self.request_set_motor(1, 1, 75.0)
                    self.request_set_motor(1, 2, set_voltage)
                return
            elif self.calibration_sweep_index == 1:
                # capture pointing from first motor, channel 2 sweep
                # Now capture the pointing information from the last voltage step.
                # There won't be a com from the next voltage step yet eventhough called, because imgs can only be
                # received by an event, and the event loop is not being processed during this blocking function.
                if not self.unknown_current_voltage:
                    # Only store captured pointing information, if I know what voltages are on the motors right now.
                    self.mot1_y_cam1_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[0]
                    self.mot1_y_cam1_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[1]
                    self.mot1_y_cam2_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[0]
                    self.mot1_y_cam2_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[1]
                else:  # Whoops never set nor get the voltage for this step, so delete this steps pointing info
                    # Reset unknown voltage flag
                    self.unknown_current_voltage = False
                    del self.mot1_y_cam1_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot1_y_cam1_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot1_y_cam2_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot1_y_cam2_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    self.num_calibration_points_dropped_current_sweep += 1
                # Keep moving calibration pointing index appropriately
                if self.calibration_pointing_index == self.num_steps_per_motor-1:
                    # Finished getting pointing information from this motor sweep
                    self.calibration_pointing_index = 0
                    # Reset number of dropped calibration points
                    self.num_calibration_points_dropped_current_sweep = 0
                    # step to next step sweep calibration
                    self.calibration_sweep_index += 1
                else:
                    # Get next step in pointing information for this sweep
                    self.calibration_pointing_index += 1
                # Go ahead and get the motor to start updating to the next set point
                if self.voltage_step < self.num_steps_per_motor:
                    # Set the next voltage step on motor current motor
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    self.voltage_step += 1
                    self.request_set_motor(1, 2, set_voltage)
                elif self.voltage_step == self.num_steps_per_motor:
                    # Finished voltage steps on the last motor, now start the next motor moving.
                    # But I still need pointing info from the last motor set position.
                    self.voltage_step = 0
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    # Set motor 1 ch2 back to center voltage
                    self.voltage_step += 1
                    self.request_set_motor(1, 2, 75.0)
                    self.request_set_motor(2, 1, set_voltage)
                return
            elif self.calibration_sweep_index == 2:
                # capture pointing from second motor, channel 1 sweep
                # Now capture the pointing information from the last voltage step.
                # There won't be a com from the next voltage step yet eventhough called, because imgs can only be
                # received by an event, and the event loop is not being processed during this blocking function.
                if not self.unknown_current_voltage:
                    # Only store captured pointing information, if I know what voltages are on the motors right now.
                    self.mot2_x_cam1_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[0]
                    self.mot2_x_cam1_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[1]
                    self.mot2_x_cam2_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[0]
                    self.mot2_x_cam2_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[1]
                else:  # Whoops never set nor get the voltage for this step, so delete this steps pointing info
                    # Reset unknown voltage flag
                    self.unknown_current_voltage = False
                    del self.mot2_x_cam1_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot2_x_cam1_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot2_x_cam2_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot2_x_cam2_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    self.num_calibration_points_dropped_current_sweep += 1
                # Keep moving calibration pointing index appropriately
                if self.calibration_pointing_index == self.num_steps_per_motor - 1:
                    # Finished getting pointing information from this motor sweep
                    self.calibration_pointing_index = 0
                    # Reset number of dropped calibration points
                    self.num_calibration_points_dropped_current_sweep = 0
                    # step to next step sweep calibration
                    self.calibration_sweep_index += 1
                else:
                    # Get next step in pointing information for this sweep
                    self.calibration_pointing_index += 1
                # Go ahead and get the motor to start updating to the next set point
                if self.voltage_step < self.num_steps_per_motor:
                    # Set the next voltage step on motor current motor
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    self.voltage_step += 1
                    self.request_set_motor(2, 1, set_voltage)
                elif self.voltage_step == self.num_steps_per_motor:
                    # Finished voltage steps on the last motor, now start the next motor moving.
                    # But I still need pointing info from the last motor set position.
                    self.voltage_step = 0
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    # Set motor 2 ch1 back to center voltage
                    self.voltage_step += 1
                    self.request_set_motor(2, 1, 75.0)
                    self.request_set_motor(2, 2, set_voltage)
                return
            elif self.calibration_sweep_index == 3:
                # capture pointing from second motor, channel 2 sweep

                # Now capture the pointing information from the last voltage step.
                # There won't be a com from the next voltage step yet eventhough called, because imgs can only be
                # received by an event, and the event loop is not being processed during this blocking function.
                if not self.unknown_current_voltage:
                    # Only store captured pointing information, if I know what voltages are on the motors right now.
                    self.mot2_y_cam1_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[0]
                    self.mot2_y_cam1_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_1_com[1]
                    self.mot2_y_cam2_x[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[0]
                    self.mot2_y_cam2_y[self.calibration_pointing_index -
                                       self.num_calibration_points_dropped_current_sweep] = self.cam_2_com[1]
                else:  # Whoops never set nor get the voltage for this step, so delete this steps pointing info
                    # Reset unknown voltage flag
                    self.unknown_current_voltage = False
                    del self.mot2_y_cam1_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot2_y_cam1_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot2_y_cam2_x[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    del self.mot2_y_cam2_y[self.calibration_pointing_index -
                                           self.num_calibration_points_dropped_current_sweep]
                    self.num_calibration_points_dropped_current_sweep += 1
                # Keep moving calibration pointing index appropriately
                if self.calibration_pointing_index == self.num_steps_per_motor - 1:
                    # Finished getting pointing information from this motor sweep
                    self.calibration_pointing_index = 0
                    # Reset number of dropped calibration points
                    self.num_calibration_points_dropped_current_sweep = 0
                    # step to next step sweep calibration, here this just ensures I do not accidentally overwrite
                    # anything if this function gets called again.
                    self.calibration_sweep_index += 1
                    # Tell update manager to calculate the calibration matrix now that the sweeps are done.
                    self.request_calculate_calibration_matrix_signal.emit()
                else:
                    # Get next step in pointing information for this sweep
                    self.calibration_pointing_index += 1
                # Go ahead and get the motor to start updating to the next set point
                if self.voltage_step < self.num_steps_per_motor:
                    # Set the next voltage step on motor current motor
                    set_voltage = self.calibration_voltages[self.voltage_step]
                    self.voltage_step += 1
                    self.request_set_motor(2, 2, set_voltage)
                elif self.voltage_step == self.num_steps_per_motor:
                    # Finished voltage steps on the last motor, now start the next motor moving.
                    # But I still need pointing info from the last motor set position.
                    self.voltage_step = 1
                    # Set motor 2 ch2 back to center voltage
                    self.request_set_motor(2, 2, 75.0)
                if self.calibration_sweep_index == 4:
                    # Return from a calibration process always to a non-locking state of the update manager.
                    self.request_toggle_lock_signal.emit(False)
                return
            elif self.calibration_pointing_index == 4:
                # If I am getting here, I am already done sweeping the motors anyway, so just do nothing.
                return
        else:
            # I only have 1 updated COM, so keep waiting for the next COM update.
            return

    @pyqtSlot()
    def calculate_calibration_matrix(self):
        # TODO: The calibration process seems bad in that the tests of the calibration fails, produces dVs way off!
        """
        Calculate the calibration matrix. And plot the resulting fits to pointing changes vs. voltage changes
        """
        """print("motor 1 x : ", self.mot1_x_voltage, len(self.mot1_x_voltage))
        print("motor 1 y : ", self.mot1_y_voltage, len(self.mot1_y_voltage))
        print("motor 2 x : ", self.mot2_x_voltage, len(self.mot2_x_voltage))
        print("motor 2 y : ", self.mot2_y_voltage, len(self.mot2_y_voltage))
        print("motor 1 x voltage, cam1 x response: ", self.mot1_x_cam1_x, len(self.mot1_x_cam1_x))
        print("motor 1 x voltage, cam1 y response: ", self.mot1_x_cam1_y, len(self.mot1_x_cam1_y))
        print("motor 1 x voltage, cam2 x response: ", self.mot1_x_cam2_x, len(self.mot1_x_cam2_x))
        print("motor 1 x voltage, cam2 y response: ", self.mot1_x_cam2_y, len(self.mot1_x_cam2_y))

        print("motor 1 y voltage, cam1 x response: ", self.mot1_y_cam1_x, " with len ", len(self.mot1_y_cam1_x))
        print("motor 1 y voltage, cam1 y response: ", self.mot1_y_cam1_y, " with len ", len(self.mot1_y_cam1_y))
        print("motor 1 y voltage, cam2 x response: ", self.mot1_y_cam2_x, " with len ", len(self.mot1_y_cam2_x))
        print("motor 1 y voltage, cam2 y response: ", self.mot1_y_cam2_y, " with len ", len(self.mot1_y_cam2_y))

        print("motor 2 x voltage, cam1 x response: ", self.mot2_x_cam1_x, " with len ", len(self.mot2_x_cam1_x))
        print("motor 2 x voltage, cam1 y response: ", self.mot2_x_cam1_y, " with len ", len(self.mot2_x_cam1_y))
        print("motor 2 x voltage, cam2 x response: ", self.mot2_x_cam2_x, " with len ", len(self.mot2_x_cam2_x))
        print("motor 2 x voltage, cam2 y response: ", self.mot2_x_cam2_y, " with len ", len(self.mot2_x_cam2_y))

        print("motor 2 y voltage, cam1 x response: ", self.mot2_y_cam1_x, " with len ", len(self.mot2_y_cam1_x))
        print("motor 2 y voltage, cam1 y response: ", self.mot2_y_cam1_y, " with len ", len(self.mot2_y_cam1_y))
        print("motor 2 y voltage, cam2 x response: ", self.mot2_y_cam2_x, " with len ", len(self.mot2_y_cam2_x))
        print("motor 2 y voltage, cam2 y response: ", self.mot2_y_cam2_y, " with len ", len(self.mot2_y_cam2_y))"""
        # calculate slopes by fitting lines
        p_mot1_x_cam1_x = np.polyfit(self.mot1_x_voltage[1:-1], self.mot1_x_cam1_x[1:-1], deg=1)
        p_mot1_x_cam2_x = np.polyfit(self.mot1_x_voltage[1:-1], self.mot1_x_cam2_x[1:-1], deg=1)
        p_mot1_x_cam1_y = np.polyfit(self.mot1_x_voltage[1:-1], self.mot1_x_cam1_y[1:-1], deg=1)
        p_mot1_x_cam2_y = np.polyfit(self.mot1_x_voltage[1:-1], self.mot1_x_cam2_y[1:-1], deg=1)

        p_mot1_y_cam1_x = np.polyfit(self.mot1_y_voltage[1:-1], self.mot1_y_cam1_x[1:-1], deg=1)
        p_mot1_y_cam2_x = np.polyfit(self.mot1_y_voltage[1:-1], self.mot1_y_cam2_x[1:-1], deg=1)
        p_mot1_y_cam1_y = np.polyfit(self.mot1_y_voltage[1:-1], self.mot1_y_cam1_y[1:-1], deg=1)
        p_mot1_y_cam2_y = np.polyfit(self.mot1_y_voltage[1:-1], self.mot1_y_cam2_y[1:-1], deg=1)

        p_mot2_x_cam1_x = np.polyfit(self.mot2_x_voltage[1:-1], self.mot2_x_cam1_x[1:-1], deg=1)
        p_mot2_x_cam2_x = np.polyfit(self.mot2_x_voltage[1:-1], self.mot2_x_cam2_x[1:-1], deg=1)
        p_mot2_x_cam1_y = np.polyfit(self.mot2_x_voltage[1:-1], self.mot2_x_cam1_y[1:-1], deg=1)
        p_mot2_x_cam2_y = np.polyfit(self.mot2_x_voltage[1:-1], self.mot2_x_cam2_y[1:-1], deg=1)

        p_mot2_y_cam1_x = np.polyfit(self.mot2_y_voltage[1:-1], self.mot2_y_cam1_x[1:-1], deg=1)
        p_mot2_y_cam2_x = np.polyfit(self.mot2_y_voltage[1:-1], self.mot2_y_cam2_x[1:-1], deg=1)
        p_mot2_y_cam1_y = np.polyfit(self.mot2_y_voltage[1:-1], self.mot2_y_cam1_y[1:-1], deg=1)
        p_mot2_y_cam2_y = np.polyfit(self.mot2_y_voltage[1:-1], self.mot2_y_cam2_y[1:-1], deg=1)
        print("Finished Polyfit.")
        # Try thresholding the slopes that are small to 0, since we anticipate that many of the degrees of freedom are
        # uncoupled:
        # if the full range of the piezo only moves this dimension by <5 pixels, m=0
        min_pixels_change_over_full_voltage_range = 0
        if np.abs(p_mot1_x_cam1_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_x_cam1_x[0] = 0
        if np.abs(p_mot1_x_cam1_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_x_cam1_y[0] = 0
        if np.abs(p_mot1_x_cam2_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_x_cam2_x[0] = 0
        if np.abs(p_mot1_x_cam2_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_x_cam2_y[0] = 0

        if np.abs(p_mot1_y_cam1_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_y_cam1_x[0] = 0
        if np.abs(p_mot1_y_cam1_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_y_cam1_y[0] = 0
        if np.abs(p_mot1_y_cam2_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_y_cam2_x[0] = 0
        if np.abs(p_mot1_y_cam2_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot1_y_cam2_y[0] = 0

        if np.abs(p_mot2_x_cam1_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_x_cam1_x[0] = 0
        if np.abs(p_mot2_x_cam1_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_x_cam1_y[0] = 0
        if np.abs(p_mot2_x_cam2_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_x_cam2_x[0] = 0
        if np.abs(p_mot2_x_cam2_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_x_cam2_y[0] = 0

        if np.abs(p_mot2_y_cam1_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_y_cam1_x[0] = 0
        if np.abs(p_mot2_y_cam1_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_y_cam1_y[0] = 0
        if np.abs(p_mot2_y_cam2_x[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_y_cam2_x[0] = 0
        if np.abs(p_mot2_y_cam2_y[0]*150) < min_pixels_change_over_full_voltage_range:
            p_mot2_y_cam2_y[0] = 0

        # Plot the pointing info as a function of voltages and the fit lines to inspect the success of calibration.
        if True:  # This just lets me collapse the below code
            import matplotlib.pyplot as plt
            # First plot the fits.
            Voltage_plot = np.linspace(0, 150, 100)
            fig, ax = plt.subplots(4, 4, dpi=200, gridspec_kw={"hspace": 0.5, "wspace": 0.3})
            ax[0, 0].plot(Voltage_plot, p_mot1_x_cam1_x[0] * Voltage_plot + p_mot1_x_cam1_x[1], linewidth = 3)
            ax[1, 0].plot(Voltage_plot, p_mot1_y_cam1_x[0] * Voltage_plot + p_mot1_y_cam1_x[1], linewidth = 3)
            ax[2, 0].plot(Voltage_plot, p_mot2_x_cam1_x[0] * Voltage_plot + p_mot2_x_cam1_x[1], linewidth = 3)
            ax[3, 0].plot(Voltage_plot, p_mot2_y_cam1_x[0] * Voltage_plot + p_mot2_y_cam1_x[1], linewidth = 3)
            ax[0, 1].plot(Voltage_plot, p_mot1_x_cam1_y[0] * Voltage_plot + p_mot1_x_cam1_y[1], linewidth = 3)
            ax[1, 1].plot(Voltage_plot, p_mot1_y_cam1_y[0] * Voltage_plot + p_mot1_y_cam1_y[1], linewidth = 3)
            ax[2, 1].plot(Voltage_plot, p_mot2_x_cam1_y[0] * Voltage_plot + p_mot2_x_cam1_y[1], linewidth = 3)
            ax[3, 1].plot(Voltage_plot, p_mot2_y_cam1_y[0] * Voltage_plot + p_mot2_y_cam1_y[1], linewidth = 3)
            ax[0, 2].plot(Voltage_plot, p_mot1_x_cam2_x[0] * Voltage_plot + p_mot1_x_cam2_x[1], linewidth = 3)
            ax[1, 2].plot(Voltage_plot, p_mot1_y_cam2_x[0] * Voltage_plot + p_mot1_y_cam2_x[1], linewidth = 3)
            ax[2, 2].plot(Voltage_plot, p_mot2_x_cam2_x[0] * Voltage_plot + p_mot2_x_cam2_x[1], linewidth = 3)
            ax[3, 2].plot(Voltage_plot, p_mot2_y_cam2_x[0] * Voltage_plot + p_mot2_y_cam2_x[1], linewidth = 3)
            ax[0, 3].plot(Voltage_plot, p_mot1_x_cam2_y[0] * Voltage_plot + p_mot1_x_cam2_y[1], linewidth = 3)
            ax[1, 3].plot(Voltage_plot, p_mot1_y_cam2_y[0] * Voltage_plot + p_mot1_y_cam2_y[1], linewidth = 3)
            ax[2, 3].plot(Voltage_plot, p_mot2_x_cam2_y[0] * Voltage_plot + p_mot2_x_cam2_y[1], linewidth = 3)
            ax[3, 3].plot(Voltage_plot, p_mot2_y_cam2_y[0] * Voltage_plot + p_mot2_y_cam2_y[1], linewidth = 3)
            # Now plot the data
            ax[0, 0].plot(self.mot1_x_voltage, self.mot1_x_cam1_x, 'r', marker='x', markersize=2)
            ax[1, 0].plot(self.mot1_y_voltage, self.mot1_y_cam1_x, 'r', marker='x', markersize=2)
            ax[2, 0].plot(self.mot2_x_voltage, self.mot2_x_cam1_x, 'r', marker='x', markersize=2)
            ax[3, 0].plot(self.mot2_y_voltage, self.mot2_y_cam1_x, 'r', marker='x', markersize=2)
            ax[0, 1].plot(self.mot1_x_voltage, self.mot1_x_cam1_y, 'r', marker='x', markersize=2)
            ax[1, 1].plot(self.mot1_y_voltage, self.mot1_y_cam1_y, 'r', marker='x', markersize=2)
            ax[2, 1].plot(self.mot2_x_voltage, self.mot2_x_cam1_y, 'r', marker='x', markersize=2)
            ax[3, 1].plot(self.mot2_y_voltage, self.mot2_y_cam1_y, 'r', marker='x', markersize=2)
            ax[0, 2].plot(self.mot1_x_voltage, self.mot1_x_cam2_x, 'r', marker='x', markersize=2)
            ax[1, 2].plot(self.mot1_y_voltage, self.mot1_y_cam2_x, 'r', marker='x', markersize=2)
            ax[2, 2].plot(self.mot2_x_voltage, self.mot2_x_cam2_x, 'r', marker='x', markersize=2)
            ax[3, 2].plot(self.mot2_y_voltage, self.mot2_y_cam2_x, 'r', marker='x', markersize=2)
            ax[0, 3].plot(self.mot1_x_voltage, self.mot1_x_cam2_y, 'r', marker='x', markersize=2)
            ax[1, 3].plot(self.mot1_y_voltage, self.mot1_y_cam2_y, 'r', marker='x', markersize=2)
            ax[2, 3].plot(self.mot2_x_voltage, self.mot2_x_cam2_y, 'r', marker='x', markersize=2)
            ax[3, 3].plot(self.mot2_y_voltage, self.mot2_y_cam2_y, 'r', marker='x', markersize=2)
            # Label the plots
            ax[0, 0].set_title("mot 1 x cam 1 x", fontsize=6)
            ax[1, 0].set_title("mot 1 y cam 1 x", fontsize=6)
            ax[2, 0].set_title("mot 2 x cam 1 x", fontsize=6)
            ax[3, 0].set_title("mot 2 y cam 1 x", fontsize=6)
            ax[0, 1].set_title("mot 1 x cam 1 y", fontsize=6)
            ax[1, 1].set_title("mot 1 y cam 1 y", fontsize=6)
            ax[2, 1].set_title("mot 2 x cam 1 y", fontsize=6)
            ax[3, 1].set_title("mot 2 y cam 1 y", fontsize=6)
            ax[0, 2].set_title("mot 1 x cam 2 x", fontsize=6)
            ax[1, 2].set_title("mot 1 y cam 2 x", fontsize=6)
            ax[2, 2].set_title("mot 2 x cam 2 x", fontsize=6)
            ax[3, 2].set_title("mot 2 y cam 2 x", fontsize=6)
            ax[0, 3].set_title("mot 1 x cam 2 y", fontsize=6)
            ax[1, 3].set_title("mot 1 y cam 2 y", fontsize=6)
            ax[2, 3].set_title("mot 2 x cam 2 y", fontsize=6)
            ax[3, 3].set_title("mot 2 y cam 2 y", fontsize=6)
            ax[3, 0].set_xlabel('Voltages (V)', fontsize=6)
            ax[3, 0].set_ylabel('position (pixels)', fontsize=6)
            # Adjust the tick parameters for better viewing size
            ax[0, 0].tick_params(axis='both', which='major', labelsize=6)
            ax[1, 0].tick_params(axis='both', which='major', labelsize=6)
            ax[2, 0].tick_params(axis='both', which='major', labelsize=6)
            ax[3, 0].tick_params(axis='both', which='major', labelsize=6)
            ax[0, 1].tick_params(axis='both', which='major', labelsize=6)
            ax[1, 1].tick_params(axis='both', which='major', labelsize=6)
            ax[2, 1].tick_params(axis='both', which='major', labelsize=6)
            ax[3, 1].tick_params(axis='both', which='major', labelsize=6)
            ax[0, 2].tick_params(axis='both', which='major', labelsize=6)
            ax[1, 2].tick_params(axis='both', which='major', labelsize=6)
            ax[2, 2].tick_params(axis='both', which='major', labelsize=6)
            ax[3, 2].tick_params(axis='both', which='major', labelsize=6)
            ax[0, 3].tick_params(axis='both', which='major', labelsize=6)
            ax[1, 3].tick_params(axis='both', which='major', labelsize=6)
            ax[2, 3].tick_params(axis='both', which='major', labelsize=6)
            ax[3, 3].tick_params(axis='both', which='major', labelsize=6)
            # Show the figure
            self.request_gui_plot_calibrate_fits.emit(fig)

        inv_calib_mat = np.array([[p_mot1_x_cam1_x[0], p_mot1_y_cam1_x[0], p_mot2_x_cam1_x[0], p_mot2_y_cam1_x[0]],
                              [p_mot1_x_cam1_y[0], p_mot1_y_cam1_y[0], p_mot2_x_cam1_y[0], p_mot2_y_cam1_y[0]],
                              [p_mot1_x_cam2_x[0], p_mot1_y_cam2_x[0], p_mot2_x_cam2_x[0], p_mot2_y_cam2_x[0]],
                              [p_mot1_x_cam2_y[0], p_mot1_y_cam2_y[0], p_mot2_x_cam2_y[0], p_mot2_y_cam2_y[0]]])
        calib_mat = np.linalg.inv(inv_calib_mat)
        # Set the calibration matrix
        self.calibration_matrix = calib_mat
        # Run some checks: see if I can reconstruct known voltage changes from dx's induced by the known voltage change.
        self.test_calibration_matrix()
        # Let the GUI thread know that calibration is done so that it can update GUI information accordingly.
        self.update_gui_new_calibration_matrix_signal.emit(calib_mat)
        print("Finished calculating calibration matrix.")
        return

    def test_calibration_matrix(self):
        """
        Test if the calibration matrix makes sense, reproduces known voltage changes from known dx's in calibration
        itself.
        """
        # First check pairs of points as dx's
        # mot1_x
        for i in range(5):
            ind_1 = np.random.randint(0, len(self.mot1_x_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot1_x_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot1_x_cam1_x[ind_2] - self.mot1_x_cam1_x[ind_1],
                                self.mot1_x_cam1_y[ind_2] - self.mot1_x_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot1_x_cam2_x[ind_2] - self.mot1_x_cam2_x[ind_1],
                                self.mot1_x_cam2_y[ind_2] - self.mot1_x_cam2_y[ind_1]])
            dx = np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find dV from dx:
            dV = np.matmul(self.calibration_matrix, dx)
            # Find what dV was to cause this dx:
            dV_known = np.array([self.mot1_x_voltage[ind_2] - self.mot1_x_voltage[ind_1], 0, 0, 0])
            # Check if they are the same:
            if np.any(np.abs(dV-dV_known) > 0.1):  # Check if any are off my more than 0.1 V
                print("Failed on motor1_X move alone. Anticipated dV ", dV_known, "calculated dV ", dV)
        # mot1_y
        for i in range(5):
            ind_1 = np.random.randint(0, len(self.mot1_y_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot1_y_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot1_y_cam1_x[ind_2] - self.mot1_y_cam1_x[ind_1],
                                self.mot1_y_cam1_y[ind_2] - self.mot1_y_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot1_y_cam2_x[ind_2] - self.mot1_y_cam2_x[ind_1],
                                self.mot1_y_cam2_y[ind_2] - self.mot1_y_cam2_y[ind_1]])
            dx = np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find dV from dx:
            dV = np.matmul(self.calibration_matrix, dx)
            # Find what dV was to cause this dx:
            dV_known = np.array([0, self.mot1_y_voltage[ind_2] - self.mot1_y_voltage[ind_1], 0, 0])
            # Check if they are the same:
            if np.any(np.abs(dV - dV_known) > 0.1):  # Check if any are off my more than 0.1 V
                print("Failed on motor1_Y move alone. Anticipated dV ", dV_known, "calculated dV ", dV)
        # mot2_x
        for i in range(5):
            ind_1 = np.random.randint(0, len(self.mot2_x_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot2_x_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot2_x_cam1_x[ind_2] - self.mot2_x_cam1_x[ind_1],
                                self.mot2_x_cam1_y[ind_2] - self.mot2_x_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot2_x_cam2_x[ind_2] - self.mot2_x_cam2_x[ind_1],
                                self.mot2_x_cam2_y[ind_2] - self.mot2_x_cam2_y[ind_1]])
            dx = np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find dV from dx:
            dV = np.matmul(self.calibration_matrix, dx)
            # Find what dV was to cause this dx:
            dV_known = np.array([0, 0, self.mot2_x_voltage[ind_2] - self.mot2_x_voltage[ind_1], 0])
            # Check if they are the same:
            if np.any(np.abs(dV - dV_known) > 0.1):  # Check if any are off my more than 0.1 V
                print("Failed on motor2_X move alone. Anticipated dV ", dV_known, "calculated dV ", dV)
        # mot2_y
        for i in range(5):
            ind_1 = np.random.randint(0, len(self.mot2_y_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot2_y_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot2_y_cam1_x[ind_2] - self.mot2_y_cam1_x[ind_1],
                                self.mot2_y_cam1_y[ind_2] - self.mot2_y_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot2_y_cam2_x[ind_2] - self.mot2_y_cam2_x[ind_1],
                                self.mot2_y_cam2_y[ind_2] - self.mot2_y_cam2_y[ind_1]])
            dx = np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find dV from dx:
            dV = np.matmul(self.calibration_matrix, dx)
            # Find what dV was to cause this dx:
            dV_known = np.array([0, 0, 0, self.mot2_y_voltage[ind_2] - self.mot2_y_voltage[ind_1]])
            # Check if they are the same:
            if np.any(np.abs(dV - dV_known) > 0.1):  # Check if any are off my more than 0.1 V
                print("Failed on motor2_Y move alone. Anticipated dV ", dV_known, "calculated dV ", dV)

        # check linear combinations of all motors changing.
        for i in range(5):
            # mot1_x
            ind_1 = np.random.randint(0, len(self.mot1_x_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot1_x_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot1_x_cam1_x[ind_2] - self.mot1_x_cam1_x[ind_1],
                                self.mot1_x_cam1_y[ind_2] - self.mot1_x_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot1_x_cam2_x[ind_2] - self.mot1_x_cam2_x[ind_1],
                                self.mot1_x_cam2_y[ind_2] - self.mot1_x_cam2_y[ind_1]])
            dx = np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find what dV was to cause this dx:
            dV_known = np.array([self.mot1_x_voltage[ind_2] - self.mot1_x_voltage[ind_1], 0, 0, 0])
            # mot1_y
            ind_1 = np.random.randint(0, len(self.mot1_y_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot1_y_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot1_y_cam1_x[ind_2] - self.mot1_y_cam1_x[ind_1],
                                self.mot1_y_cam1_y[ind_2] - self.mot1_y_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot1_y_cam2_x[ind_2] - self.mot1_y_cam2_x[ind_1],
                                self.mot1_y_cam2_y[ind_2] - self.mot1_y_cam2_y[ind_1]])
            dx += np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find what dV was to cause this dx:
            dV_known += np.array([0, self.mot1_y_voltage[ind_2] - self.mot1_y_voltage[ind_1], 0, 0])
            # mot2_x
            ind_1 = np.random.randint(0, len(self.mot2_x_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot2_x_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot2_x_cam1_x[ind_2] - self.mot2_x_cam1_x[ind_1],
                                self.mot2_x_cam1_y[ind_2] - self.mot2_x_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot2_x_cam2_x[ind_2] - self.mot2_x_cam2_x[ind_1],
                                self.mot2_x_cam2_y[ind_2] - self.mot2_x_cam2_y[ind_1]])
            dx += np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find what dV was to cause this dx:
            dV_known += np.array([0, 0, self.mot2_x_voltage[ind_2] - self.mot2_x_voltage[ind_1], 0])
            # mot2_y
            ind_1 = np.random.randint(0, len(self.mot2_y_cam1_x))
            ind_2 = np.random.randint(0, len(self.mot2_y_cam1_x))
            # Construct a dx as my code does, from a known voltage change.
            dx_cam1 = np.array([self.mot2_y_cam1_x[ind_2] - self.mot2_y_cam1_x[ind_1],
                                self.mot2_y_cam1_y[ind_2] - self.mot2_y_cam1_y[ind_1]])
            dx_cam2 = np.array([self.mot2_y_cam2_x[ind_2] - self.mot2_y_cam2_x[ind_1],
                                self.mot2_y_cam2_y[ind_2] - self.mot2_y_cam2_y[ind_1]])
            dx += np.concatenate((dx_cam1, dx_cam2), axis=0)
            # Find what dV was to cause this dx:
            dV_known += np.array([0, 0, 0, self.mot2_y_voltage[ind_2] - self.mot2_y_voltage[ind_1]])

            # Find dV from overall dx:
            dV = np.matmul(self.calibration_matrix, dx)
            if np.any(np.abs(dV - dV_known) > 0.1):  # Check if any are off my more than 0.1 V
                print("Failed on linear combination of dx. Anticipated dV ", dV_known, "calculated dV ", dV)

        return

    # Analysis/convenience methods.
    def store_data(self, state, IR):
        """
        Store pointing data for analysis later.
        """
        # TODO: Reimplement storing data.
        if IR:
            if state == 0:
                state = "_Measure_IR"
            elif state == 1:
                state = "_Calibrate_IR"
            elif state == 2:
                state = "_Locked_IR"
            elif state == 3:
                state = "_Align_IR"
        else:
            if state == 0:
                state = "_Measure_Vis"
            elif state == 1:
                state = "_Calibrate_Vis"
            elif state == 2:
                state = "_Locked_Vis"
            elif state == 3:
                state = "_Align_Vis"
        date = str(np.datetime64('today', 'D'))
        if len(self.dx) > 0:
            filename = 'Data/' + date + state + '_dx'
            np.savetxt(filename, self.dx, fmt='%f')
        if len(self.t1) > 0:
            filename = 'Data/' + date + state + '_t1'
            np.savetxt(filename, self.t1, fmt='%f')
        if len(self.t2) > 0:
            filename = 'Data/' + date + state + '_t2'
            np.savetxt(filename, self.t2, fmt='%f')
        return

    def reset_data(self):
        """
        Re-init dx, t1, and t2 to empty
        """
        self._dx = []
        self._t1 = []
        self._t2 = []
        return

    def load_data(self, dx, t1, t2):
        """
        Set dx, t1, t2 from loaded data. This allows convenient analysis of pointing data outside of the app.
        """
        self._dx = dx
        self._t1 = t1
        self._t2 = t2
        return

    @property
    def frequency_domain(self):
        return self._frequency_domain

    @frequency_domain.setter
    def frequency_domain(self, list):
        """
        Nu is in Hz
        list[0] is Nu for cam 1 dx[:,0:2]
        list[1] is Nu for cam 2 dx[:,2,:]
        """
        self._frequency_domain = list
        return

    @property
    def frequency_data(self):
        """
        Find the frequency data on a uniformly spaced grid from a non-uniformly sampled time domain.
        The NyQuist freq >= the (2 * minnimum dt)^-1 in the time-domain signal.
        The spacing in frequency space is defined by the time window, T.
        So, with dt, we can find the number of points we expect if we sample up to +/-1/2dt_min
        With the above, we can find Nu in 1/[unit time], which is ms here; so scale Nu by 1000 to yield Hz.
        Finally to satisfy Parseval's theorem, we need to scale the resulting frequency data by dt.
        Oh, and we scaled the time coordinate by the time window, because nfft wants -1/2 to 1/2 domain. We choose to
        define t=0 at the center of the time window. A shift in time just adds a linear phase in frequency domain.

        Set the frequency domain property and return a list of frequency data.
        """
        dt1 = self.t1[1:] - self.t1[0:-1]
        dt1_min = np.amin(dt1)
        dt2 = self.t2[1:] - self.t2[0:-1]
        dt2_min = np.amin(dt2)
        T1 = self.t1[-1] - self.t1[0]
        T2 = self.t2[-1] - self.t2[0]
        time1 = (self.t1 - np.amin(self.t1)) - T1 / 2  # t=0 at the center of the data
        time2 = (self.t2 - np.amin(self.t2)) - T2 / 2  # t=0 at the center of the data
        N1 = int(np.floor(T1 / dt1_min))
        if N1 % 2 == 1:
            N1 = N1 - 1
        N2 = int(np.floor(T2 / dt2_min))
        if N2 % 2 == 1:
            N2 = N2 - 1
        dNu1 = 1 / T1
        dNu2 = 1 / T2
        Nu1 = np.linspace(-N1 / 2, N1 / 2 - 1, N1) * dNu1 * 1000.0  # Hz
        Nu2 = np.linspace(-N2 / 2, N2 / 2 - 1, N2) * dNu2 * 1000.0  # Hz
        self.frequency_domain = [Nu1, Nu2]
        FT_cam1_dx = nfft.nfft_adjoint(time1 / T1, self.dx[:, 0], N1) * T1 / len(self.t1)
        FT_cam1_dy = nfft.nfft_adjoint(time1 / T1, self.dx[:, 1], N1) * T1 / len(self.t1)
        FT_cam2_dx = nfft.nfft_adjoint(time2 / T2, self.dx[:, 2], N2) * T2 / len(self.t2)
        FT_cam2_dy = nfft.nfft_adjoint(time2 / T2, self.dx[:, 3], N2) * T2 / len(self.t2)
        return [FT_cam1_dx, FT_cam1_dy, FT_cam2_dx, FT_cam2_dy]

    @property
    def standard_deviation(self):
        """
        Calculate the standard deviation the dx vector.
        """
        return np.nanstd(self.dx, axis=0)

    # Camera related properties and methods.
    @pyqtSlot(int, np.ndarray)
    def update_r0(self, cam_num: int, r0: np.ndarray):
        if cam_num == 1:
            self._r0[0:2] = r0
        elif cam_num == 2:
            self._r0[2:] = r0
        return

    @pyqtSlot(int, float)
    def update_img_thresholds(self, cam_num: int, threshold: float):
        if cam_num == 1:
            self.img1_threshold = threshold
        elif cam_num == 2:
            self.img2_threshold = threshold
        return

    @property
    def cam_1_img(self):
        return self._cam_1_img

    @cam_1_img.setter
    def cam_1_img(self, img):
        """
        img from camera 1. Also inform threads that new img is received.
        """
        self._cam_1_img = img
        self.cam_img_received_signal.emit(1)
        return

    @staticmethod
    def find_com(img_thresh):
        """
        Find the COM of an thresholded image as returned by cv2.threshold
        """
        M = cv2.moments(img_thresh)
        if M['m00'] != 0:
            com_x = M["m01"] / M["m00"]
            com_y = M["m10"] / M["m00"]
            return com_x, com_y
        return None, None

    @pyqtSlot(int, np.ndarray, float)
    def process_img(self, cam_number: int, img: np.ndarray, timestamp: float):
        """
        Given an image, self.cam_x_img, calculate the center of mass in pixel coordinates
        For now, just find COM, but in the future, do any preprocessing that I want.
        """
        if cam_number == 1:
            cv2.subtract(img, self.img1_threshold, img)  # Because I am using uint, any negative result is set to 0
            com_x, com_y = self.find_com(img)
            if com_x is not None:
                com_x += self._r0[0]
                com_y += self._r0[1]
                self.cam1_dx = np.asarray([com_x, com_y]) - self.set_pos[0:2]
                self.t1 = timestamp
                self.cam_1_com = np.asarray([com_x, com_y])
                self.update_gui_cam_com_signal.emit(1, np.asarray([com_x, com_y]))
            if self._cam1_img_count >= self._update_GUI_images_every_n_images:
                self.update_gui_img_signal.emit(1, np.asarray(img))
                self._cam1_img_count = 0
            else:
                self._cam1_img_count += 1
        elif cam_number == 2:
            cv2.subtract(img, self.img2_threshold, img)  # Because I am using uint, any negative result is set to 0
            com_x, com_y = self.find_com(img)
            if com_x is not None:
                com_x += self._r0[2]
                com_y += self._r0[3]
                self.cam2_dx = np.asarray([com_x, com_y]) - self.set_pos[2:]
                self.t2 = timestamp
                self.cam_2_com = np.asarray([com_x, com_y])
                self.update_gui_cam_com_signal.emit(2, np.asarray([com_x, com_y]))
            if self._cam2_img_count >= self._update_GUI_images_every_n_images:
                self.update_gui_img_signal.emit(2, np.asarray(img))
                self._cam2_img_count = 0
            else:
                self._cam2_img_count += 1
        return

    @pyqtSlot(int)
    def update_num_cameras_connected(self, increment_num_cameras: int):
        """
        Updates the update manager with knowledge of how many cameras are connected.
        """
        self.num_cams_connected += increment_num_cameras
        if self.num_cams_connected < 0:
            raise Exception("Somehow Update Manager now believes there are - cameras connected?")
        elif self.num_cams_connected > 2:
            raise Exception("Somehow Update Manager now believes there are >2 cameras connected?")
        return

    @property
    def cam_2_img(self):
        return self._cam_2_img

    @cam_2_img.setter
    def cam_2_img(self, img):
        """
        img from camera 2. Also inform threads that new img is received.
        """
        self._cam_2_img = img
        self.cam_img_received_signal.emit(2)
        return

    @property
    def t1(self):
        """Camera 1 image timestamp units of ms."""
        return np.asarray(self._t1)

    @t1.setter
    def t1(self, value):
        self._t1.append(value)
        if len(self._t1) > 100:
            del self._t1[0]
        return

    @property
    def t2(self):
        """Camera 2 image timestamp units of ms"""
        return np.asarray(self._t2)

    @t2.setter
    def t2(self, value):
        self._t2.append(value)
        if len(self._t2) > 100:
            del self._t2[0]
        return

    @property
    def cam_1_com(self):
        return self._cam_1_com

    @cam_1_com.setter
    def cam_1_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 1
        """
        if self._motors_updated:
            if self.cam1_time_motors_updated is None:
                self.cam1_time_motors_updated = self.t1[-1]
            if self._calibrating:
                if self.cam1_com_avg_num == 1:
                    self._cam_1_com = vector/self.num_frames_to_average_during_calibrate
                    self.cam1_com_avg_num += 1
                    return
                elif self.cam1_com_avg_num < self.num_frames_to_average_during_calibrate:
                    self._cam_1_com += vector/self.num_frames_to_average_during_calibrate
                    self.cam1_com_avg_num += 1
                    return
                elif self.cam1_com_avg_num == self.num_frames_to_average_during_calibrate:
                    self._cam_1_com += vector / self.num_frames_to_average_during_calibrate
                    check = np.sum(np.sqrt(np.square(self._cam_1_com-vector)))
                    if check > 4:
                        print("Averaged com - current com magnitude is ", check)
                    self.cam1_com_avg_num += 1
                else:
                    # Do not allow the num_frames... + 1th frame to be included. Always only num_frames...
                    return
            self._cam1_com_updated = True
            self.com_found_signal.emit()
        else:
            pass
            # print("motors not updated, ",
            #       self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated, self.motor2_ch2_updated)
        return

    @property
    def cam_2_com(self):
        return self._cam_2_com

    @cam_2_com.setter
    def cam_2_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 2
        """
        if self._motors_updated:
            if self.cam2_time_motors_updated is None:
                self.cam2_time_motors_updated = self.t2[-1]
            if self._calibrating:
                if self.cam2_com_avg_num == 1:
                    self._cam_2_com = vector / self.num_frames_to_average_during_calibrate
                    self.cam2_com_avg_num += 1
                    return
                elif self.cam2_com_avg_num < self.num_frames_to_average_during_calibrate:
                    self._cam_2_com += vector / self.num_frames_to_average_during_calibrate
                    self.cam2_com_avg_num += 1
                    return
                elif self.cam2_com_avg_num == self.num_frames_to_average_during_calibrate:
                    self._cam_2_com += vector / self.num_frames_to_average_during_calibrate
                    check = np.sum(np.sqrt(np.square(self._cam_2_com - vector)))
                    if check > 4:
                        print("Averaged com - current com magnitude is ", check)
                    self.cam2_com_avg_num += 1
                else:
                    # Do not allow the num_frames... + 1th frame to be included. Always only num_frames...
                    return
            self._cam2_com_updated = True
            self.com_found_signal.emit()
        else:
            pass
            # print("motors not updated, ",
            #       self.motor1_ch1_updated, self.motor1_ch2_updated, self.motor2_ch1_updated, self.motor2_ch2_updated)
        return

    def com(self):
        return np.concatenate((self.cam_1_com, self.cam_2_com), axis=0)

    @property
    def cam1_dx(self):
        return np.asarray(self._cam1_dx)

    @cam1_dx.setter
    def cam1_dx(self, vector):
        # [row, column]
        self._cam1_dx.append(vector)
        if len(self._cam1_dx) > 100:
            del self._cam1_dx[0]
        return

    @property
    def cam2_dx(self):
        return np.asarray(self._cam2_dx)

    @cam2_dx.setter
    def cam2_dx(self, vector):
        # [row, column]
        self._cam2_dx.append(vector)
        if len(self._cam2_dx) > 100:
            del self._cam2_dx[0]
        return

    # Misc. properties
    @property
    def update_voltage(self):
        """
        Store the update voltage to be applied.
        """
        return self._update_voltage

    @update_voltage.setter
    def update_voltage(self, vector):
        self._update_voltage = vector
        return

    @property
    def dV(self):
        """
        Change in voltage for the next update.
        """
        return self._dV

    @dV.setter
    def dV(self, vector):
        self._dV = vector
        return

    @property
    def dx(self):
        """
        Returns dx as a matrix, where row number corresponds to t, and columns to positions as defined in the
        setter method below.
        """
        return np.concatenate((self.cam1_dx, self.cam2_dx), axis=1)

    @property
    def V0(self):
        return self._V0

    @V0.setter
    def V0(self, vector):
        """
        starting voltage while calculating the update voltage.
        """
        self._V0 = vector
        return

    @property
    def calibration_matrix(self):
        return self._calibration_matrix

    @calibration_matrix.setter
    def calibration_matrix(self, matrix):
        """
        Understand that this matrix is dV_i/dx_j where ij indexes like a normal matrix, i.e. row column.
        Therefore, dV = calib_mat * dx where dx is a column vector, [d_cam1_x, d_cam1_y, d_cam2_x, d_cam2_y].Transpose,
        where, for example, d_cam1_x is the difference in the desired position of cam1_x and the calculated COM x_coordinate of cam 1.
        and dV is a column vector, [d_mot1_x, d_mot1_y, d_mot2_x, d_mot2_y,].Transpose, i.e. the change in voltage
         on each motor to bring the COM coordinates to desired position.
        Therefore, this matrix can be used to update the motor voltages as New_V = Old_V - calib_mat*dx
        """
        # TODO: Are my vecotrs making sense? Row vs Column?
        self._calibration_matrix = matrix
        return

    @property
    def set_pos(self):
        return self._set_pos

    @set_pos.setter
    def set_pos(self, vector):
        """
        Desired laser pointing—target to move towards.
        """
        self._set_pos = vector
        return

    @pyqtSlot()
    def close(self):
        """
        Request to close down the UpdateManager: Requires, deleting motors and stopping and deleting motor threads
        """
        # Quit motor threads.
        if self.motor1_thread is not None:
            self.motor1_thread.quit()
        if self.motor2_thread is not None:
            self.motor2_thread.quit()
        # Wait for threads to return
        if self.motor1_thread is not None:
            self.motor1_thread.wait()
        if self.motor2_thread is not None:
            self.motor2_thread.wait()

        # Motor threads are not running. Nothing directly calls the motors, thus nothing will simultaneously be
        # interacting with my motor objects. So, I can call them directly. No race concerns.

        # Close any motor objects
        if self.motor1 is not None:
            self.motor1._app_closing = True
            self.motor1.close()
            del self.motor1
            del self.motor1_thread
        if self.motor2 is not None:
            self.motor2._app_closing = True
            self.motor2.close()
            del self.motor2
            del self.motor2_thread
        return

class PIDUpdateManager(UpdateManager):

    request_update_cam_exposure_time = pyqtSignal(int, float)
    request_update_pid_settings = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self._dt = None
        self._integral_ti = np.zeros(4)
        self._P = 0.5
        self._I = 0.1
        self._D = 0
        self.is_PID = True
        self._use_PID = True
        self.cam1_exp_time = None
        self.cam2_exp_time = None
        self.cam1_time_last_found_int = 0
        self.cam2_time_last_found_int = 0

    def connect_signals(self):
        super().connect_signals()
        self.request_update_cam_exposure_time.connect(self.set_cam_exp_time)
        self.request_update_pid_settings.connect(lambda P, I, D: setattr(self, "P", P))
        self.request_update_pid_settings.connect(lambda P, I, D: setattr(self, "I", I))
        self.request_update_pid_settings.connect(lambda P, I, D: setattr(self, "D", D))

    @pyqtSlot(bool)
    def lock_pointing(self, lock: bool):
        """
        Additionally, need to clear integral_ti if starting to lock again.
        """
        if lock:
            self.integral_ti = np.zeros(4)
        super().lock_pointing(lock)
        return

    def calc_update_dx(self):
        """
        Find the effective dx to use when calculating the update voltages. Here is where we turn this into a PID
        controller by making the update_dx a sum of a P, I, and D term.
        """
        # PID only makes sense if there are at least 2 data points. If only one, just do P.
        if len(self.t1) > 1 and self._use_PID:
            super().calc_update_dx()  # Averages all dx, since the motors were updated.
            self.update_dx *= self.P
            # prep for I and D terms:
            derivatives = self.calc_derivative()
            self.calc_integral()
            # Add the I term:
            self.update_dx += self.I*self.integral_ti
            # Add the D term:
            self.update_dx += self.D*derivatives
            # Find dX weighted total from PID.
            return
        else:
            super().calc_update_dx()
            self.update_dx *= self.P
            return
        return

    def fit_update(self):
        """
        When the update voltages are out of their allowed range, I want to try to find an update that minimizes
        my dx in the future step, constrained by the allowed piezo ranges. That is what this function does.

        This method works, but it does not perform well when the voltages are out of range... which is its purpose...
        SO, maybe in the future, I can try moving the target set point such that the voltages are in range but the
        set point is as close as possible. Then, maybe the updates can be stable... But not today!
        """
        super().fit_update()
        self.update_voltage *= self.P
        return

    def calc_integral(self):
        # Currently using the most simplistic numerical integral, may want to improve
        inds_to_update_integral_with = np.where(self.t1 > self.cam1_time_last_found_int)
        self.cam1_time_last_found_int = self.t1[-1]
        # units of pixels*ms
        increment_cam1 = np.sum(self.cam1_exp_time*self.cam1_dx[inds_to_update_integral_with], axis=0)
        inds_to_update_integral_with = np.where(self.t2 > self.cam2_time_last_found_int)
        self.cam2_time_last_found_int = self.t2[-1]
        # units of pixels*ms
        increment_cam2 = np.sum(self.cam2_exp_time * self.cam2_dx[inds_to_update_integral_with], axis=0)
        self._integral_ti += np.concatenate([increment_cam1, increment_cam2], axis=0)
        return

    def calc_derivative(self):
        # TODO: I need at least two frames post update motors.
        inds_post_motor_update = np.where(self.t1 >= self.cam1_time_motors_updated)
        del inds_post_motor_update[-1]
        dx_derivative_cam1_avg = np.average((self.cam1_dx[inds_post_motor_update+1] -
                                             self.cam1_dx[inds_post_motor_update]) /
                                            (self.t1[inds_post_motor_update+1] - self.t1[inds_post_motor_update]),
                                            axis=0)
        inds_post_motor_update = np.where(self.t2 >= self.cam2_time_motors_updated)
        del inds_post_motor_update[-1]
        dx_derivative_cam2_avg = np.average((self.cam2_dx[inds_post_motor_update + 1] -
                                             self.cam2_dx[inds_post_motor_update]) /
                                            (self.t2[inds_post_motor_update + 1] - self.t2[inds_post_motor_update]),
                                            axis=0)
        return np.concatenate([dx_derivative_cam1_avg, dx_derivative_cam2_avg], axis=0)

    @property
    def integral_ti(self):
        return self._integral_ti

    @integral_ti.setter
    def integral_ti(self, vector):
        self._integral_ti = vector
        return

    @pyqtSlot(int, float)
    def set_cam_exp_time(self, cam_num: int, cam_exp: float):
        if cam_num == 1:
            self.cam1_exp_time = cam_exp
        elif cam_num == 2:
            self.cam2_exp_time = cam_exp
        return

    @property
    def P(self):
        return self._P

    @P.setter
    def P(self, value):
        self._P = value
        return

    @property
    def I(self):
        return self._I

    @I.setter
    def I(self, value):
        self._I = value
        return

    @property
    def D(self):
        return self._D

    @D.setter
    def D(self, value):
        self._D = value
        return