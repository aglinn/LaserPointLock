from Thorlabs_MDT69XB_PythonSDK import MDT_COMMAND_LIB as mdt
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot


class BaseMotor(QObject):
    """
    This class is meant to be inherited by any piezo motor controller. It provides the signals and slots required to
    operate with the program. It will be written for the MDT693B, such that the MDT693B need only inherit from this
    class and add additional non-universal init procedures, but this class will not operate without the appropriate
    additional init steps. In general, methods should be overwritten as needed by other motor.
    Subclass QObject. Create signals and connect them to slots.
    """

    # first int on any emitted signals should be motor number.
    # If there is a second int, it indicates the channel number.
    # Motor Signals
    close_complete_signal = pyqtSignal(int)
    close_fail_signal = pyqtSignal(int)
    # Ch1 signals
    set_ch1V_complete_signal = pyqtSignal(int, int, float)
    set_ch1V_failed_signal = pyqtSignal(int, int)
    get_ch1V_failed_signal = pyqtSignal(int, int)
    returning_ch1V_signal = pyqtSignal(int, int, float)
    # Ch2 signals
    set_ch2V_complete_signal = pyqtSignal(int, int, float)
    set_ch2V_failed_signal = pyqtSignal(int, int)
    get_ch2V_failed_signal =pyqtSignal(int, int)
    returning_ch2V_signal = pyqtSignal(int, int, float)
    # Ch3 signals
    set_ch3V_complete_signal = pyqtSignal(int, int, float)
    set_ch3V_failed_signal = pyqtSignal(int, int)
    get_ch3V_failed_signal = pyqtSignal(int, int)
    returning_ch3V_signal = pyqtSignal(int, int, float)

    def __init__(self, motor_number, ch1: str = 'X', ch2: str = 'Y', ch3: str = 'Z'):
        """
        This constructor only does the universal steps of labeling channel 1,2,3 as X, Y, Z, and assigns a motor number.

        A full motor constructor opens the motor, sets the appropriate voltage bounds, and sets the voltage increment.
        And whatever else your motor needs.
        """
        # Init the QObject.
        super().__init__()
        # Assign which motor number this motor is:
        self.motor_number = motor_number
        # Assign number to letter mapping.
        self._ch1 = ch1
        self._ch2 = ch2
        self._ch3 = ch3
        # Initialize properties for holding voltages on each channel.
        self._ch1_v = [0]
        self._ch2_v = [0]
        self._ch3_v = [0]
        # The app is not closing upon opening, changed when actually closing to assisst with shutdown procedures.
        self._app_closing = False
        return

    @pyqtSlot()
    def close(self):
        """
        Close connection to port/release hardware, emit close return status (success/fail), and put deletion on the
        stack if the app is not closing. If closing, then the thread is already stopped, thus deletion is handled
        elsewhere.
        """
        ret = mdt.mdtClose(self.handle)
        if ret < 0:
            # Close failed
            self.close_fail_signal.emit(self.motor_number)
            raise Exception("Motor did not close.")
        else:
            # Close successful
            print("closed a mdt motor number ", self.motor_number)
            self.close_complete_signal.emit(self.motor_number)
            if not self._app_closing:
                self.deleteLater()
            return

    @property
    def ch1(self):
        return self._ch1

    @property
    def ch2(self):
        return self._ch2

    @property
    def ch3(self):
        return self._ch3

    @ch1.setter
    def ch1(self, val: str):
        self._ch1 = val

    @ch2.setter
    def ch2(self, val: str):
        self._ch2 = val

    @ch3.setter
    def ch3(self, val: str):
        self._ch3 = val

    @pyqtSlot()
    def ch1_v(self):
        """
        method to read the voltage on channel 1 of the motor. Value returned and also signals emitted.
        """
        if self.ch1 == 'X' or self.ch1 == 'x':
            ret = mdt.mdtGetXAxisVoltage(self.handle, self._ch1_v)
        elif self.ch1 == 'Y' or self.ch1 == 'y':
            ret = mdt.mdtGetYAxisVoltage(self.handle, self._ch1_v)
        elif self.ch1 == 'Z' or self.ch1 == 'z':
            ret = mdt.mdtGetZAxisVoltage(self.handle, self._ch1_v)
        if ret >= 0:
            self.returning_ch1V_signal.emit(self.motor_number, 1, self._ch1_v[0])
        else:
            print("Warning: The ch1,", self.ch1, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not read correctly.")
            self.get_ch1V_failed_signal.emit(self.motor_number, 1)
        return self._ch1_v[0]

    @pyqtSlot()
    def ch2_v(self):
        """
        method to read the voltage on channel 2 of the motor. Value returned and also signals emitted.
        """
        if self.ch2 == 'X' or self.ch2 == 'x':
            ret = mdt.mdtGetXAxisVoltage(self.handle, self._ch2_v)
        elif self.ch2 == 'Y' or self.ch2 == 'y':
            ret = mdt.mdtGetYAxisVoltage(self.handle, self._ch2_v)
        elif self.ch2 == 'Z' or self.ch2 == 'z':
            ret = mdt.mdtGetZAxisVoltage(self.handle, self._ch2_v)
        if ret >= 0:
            self.returning_ch2V_signal.emit(self.motor_number, 2, self._ch2_v[0])
        else:
            print("Warning: The voltage on ch2,", self.ch2, ", on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not read correctly.")
            self.get_ch2V_failed_signal.emit(self.motor_number, 2)
        return self._ch2_v[0]

    @pyqtSlot()
    def ch3_v(self):
        """
        method to read the voltage on channel 3 of the motor. Value returned and also signals emitted.
        """
        if self.ch3 == 'X' or self.ch3 == 'x':
            ret = mdt.mdtGetXAxisVoltage(self.handle, self._ch3_v)
        elif self.ch3 == 'Y' or self.ch3 == 'y':
            ret = mdt.mdtGetYAxisVoltage(self.handle, self._ch3_v)
        elif self.ch3 == 'Z' or self.ch3 == 'z':
            ret = mdt.mdtGetZAxisVoltage(self.handle, self._ch3_v)
        if ret >= 0:
            self.returning_ch3V_signal.emit(self.motor_number, 3, self._ch3_v[0])
        else:
            print("Warning: The voltage on ch3,", self.ch3, ", on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not read correctly.")
            self.get_ch3V_failed_signal.emit(self.motor_number, 3)
        return self._ch3_v[0]

    @pyqtSlot(float)
    def set_ch1_v(self, v: float):
        """
        Set the voltage on motor channel number 1 to v. Emit signal to inform success or failure and set value.
        """
        v = np.round(v, 3)
        if v == 0:
            v = 0.0
        if self.ch1 == 'X' or self.ch1 == 'x':
            rep = mdt.mdtSetXAxisVoltage(self.handle, v)
        elif self.ch1 == 'Y' or self.ch1 == 'y':
            rep = mdt.mdtSetYAxisVoltage(self.handle, v)
        elif self.ch1 == 'Z' or self.ch1 == 'z':
            rep = mdt.mdtSetZAxisVoltage(self.handle, v)
        if rep >= 0:
            self.set_ch1V_complete_signal.emit(self.motor_number, 1, v)
        else:
            print("Warning: The ch1,", self.ch1, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not SET correctly. attempted to set ", v)
            self.set_ch1V_failed_signal.emit(self.motor_number, 1)
        return

    @pyqtSlot(float)
    def set_ch2_v(self, v: float):
        """
        Set the voltage on motor channel number 2 to v. Emit signal to inform success or failure and set value.
        """
        v = np.round(v, 3)
        if v == 0:
            v = 0.0
        if self.ch2 == 'X' or self.ch2 == 'x':
            rep = mdt.mdtSetXAxisVoltage(self.handle, v)
        elif self.ch2 == 'Y' or self.ch2 == 'y':
            rep = mdt.mdtSetYAxisVoltage(self.handle, v)
        elif self.ch2 == 'Z' or self.ch2 == 'z':
            rep = mdt.mdtSetZAxisVoltage(self.handle, v)
        if rep >= 0:
            self.set_ch2V_complete_signal.emit(self.motor_number, 2, v)
        else:
            print("Warning: The ch2,", self.ch2, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not SET correctly. Attempted to set", v)
            self.set_ch2V_failed_signal.emit(self.motor_number, 2)
        return

    @pyqtSlot(float)
    def set_ch3_v(self, v: float):
        """
        Set the voltage on motor channel number 3 to v. Emit signal to inform success or failure and set value.
        """
        v = np.round(v, 3)
        if v == 0:
            v = 0.0
        if self.ch3 == 'X' or self.ch3 == 'x':
            rep = mdt.mdtSetXAxisVoltage(self.handle, v)
        elif self.ch3 == 'Y' or self.ch3 == 'y':
            rep = mdt.mdtSetYAxisVoltage(self.handle, v)
        elif self.ch3 == 'Z' or self.ch3 == 'z':
            rep = mdt.mdtSetZAxisVoltage(self.handle, v)
        if rep >= 0:
            self.set_ch3V_complete_signal.emit(self.motor_number, 3, v)
        else:
            print("Warning: The ch3,", self.ch3, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not SET correctly. Attempted to set", v)
            self.set_ch3V_failed_signal.emit(self.motor_number, 3)
        return

    def get_info(self, id):
        """
        Get any desired info on the motor being used.
        """
        ret = mdt.mdtGetId(self.handle, id)
        if ret < 0:
            print("Warning: Could not get the id from the motor with serial number = ", self._serial_number)
        return