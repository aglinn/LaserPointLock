import pyvisa as visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from pyvisa.errors import InvalidSession
from typing import NewType
import re
import numpy as np
from PyQt5.QtCore import pyqtSlot
from packages.motors.base import BaseMotor

VisaResourceManager = NewType('VisaResourceManager', visa.ResourceManager)


class MDT693AMotor(BaseMotor):
    """
    Code that implements a Motor assuming a connection to the ThorLabs MDT693A
    """

    def __init__(self, rm: VisaResourceManager, motor_number, com_port: str, ch1: str = 'X', ch2: str = 'Y',
                 ch3: str = 'Z'):
        """
        rm is a pyvisa ResourceManager
        motor_number is assigned for communicating with other QObjects.
        com_port is the RS232 com port for the motor
        ch1 should be a string that helps direct the motor to control the software X
        ch2 should be a string that helps direct the motor to control the software Y
        ch3 should be a string that helps direct the motor to control the software Z
        """
        # Baud Rate: 115200
        # Date bits: 8
        # Parity: none
        # Stop Bits: 1
        # Flow control: none
        self.port = com_port
        self.inst = rm.open_resource(com_port, baud_rate=115200, data_bits=8, parity=Parity.none,
                                     flow_control=VI_ASRL_FLOW_NONE, stop_bits=StopBits.one)
        # Init the Base motor object.
        super().__init__(motor_number, ch1, ch2, ch3)
        # Set voltage high and low to 150 and 0.
        # Low to 0
        cmd = self.ch1 + 'L' + str('{:.1f}'.format(0))
        rep = self.inst.query(cmd)
        cmd = self.ch1+'L?'
        rep = self.inst.query(cmd)
        rep = re.findall("\d+", rep)
        if not np.abs(float(rep[0])) < 0.2:
            print("Warning channel, ", self.ch1, ", on motor number, ", self.motor_number, ", did not set low V lim "
                                                                                           "correctly.")
        cmd = self.ch2 + 'L' + str('{:.1f}'.format(0))
        rep = self.inst.query(cmd)
        cmd = self.ch2 + 'L?'
        rep = re.findall("\d+", self.inst.query(cmd))
        if not np.abs(float(rep[0])) < 0.2:
            print("Warning channel, ", self.ch2, ", on motor number, ", self.motor_number, ", did not set low V lim "
                                                                                           "correctly.")
        cmd = self.ch3 + 'L' + str('{:.1f}'.format(0))
        rep = self.inst.query(cmd)
        cmd = self.ch3 + 'L?'
        rep = re.findall("\d+", self.inst.query(cmd))
        if not np.abs(float(rep[0])) < 0.2:
            print("Warning channel, ", self.ch3, ", on motor number, ", self.motor_number, ", did not set low V lim "
                                                                                           "correctly.")
        # High to 150
        cmd = self.ch1 + 'H' + str('{:.1f}'.format(150.0))
        rep = self.inst.query(cmd)
        cmd = self.ch1 + 'H?'
        rep = re.findall("\d+", self.inst.query(cmd))
        if not np.abs(float(rep[0])-150.0) < 0.2:
            print("Warning channel, ", self.ch1, ", on motor number, ", self.motor_number, ", did not set High V lim "
                                                                                           "correctly.")
        cmd = self.ch2 + 'H' + str('{:.1f}'.format(150.0))
        rep = self.inst.query(cmd)
        cmd = self.ch2 + 'H?'
        rep = re.findall("\d+", self.inst.query(cmd))
        if not np.abs(float(rep[0]) - 150.0) < 0.2:
            print("Warning channel, ", self.ch2, ", on motor number, ", self.motor_number, ", did not set High V lim "
                                                                                           "correctly.")
        cmd = self.ch3 + 'H' + str('{:.1f}'.format(150.0))
        rep = self.inst.query(cmd)
        cmd = self.ch3 + 'H?'
        rep = re.findall("\d+", self.inst.query(cmd))
        if not np.abs(float(rep[0]) - 150.0) < 0.2:
            print("Warning channel, ", self.ch3, ", on motor number, ", self.motor_number, ", did not set High V lim "
                                                                                           "correctly.")
        # Now assign the properties the actual starting values.
        self._ch1_v = [self.ch1_v]
        self._ch2_v = [self.ch2_v]
        self._ch3_v = [self.ch3_v]
        self._id = self.get_info()
        return

    def close(self):
        """
        Close connection to port/release hardware, emit close return status (success/fail), and put deletion on the
        stack if the app is not closing. If closing, then the thread is already stopped, thus deletion is handled
        elsewhere.
        """
        # Close connection to port
        self.inst.close()
        try:
            _ = self.inst.session
            # Close failed
            self.close_fail_signal.emit(self.motor_number)
            raise Exception("Motor did not close.")
        except InvalidSession:
            # Close successful
            print("closed a mdt motor number ", self.motor_number)
            self.close_complete_signal.emit(self.motor_number)
            if not self._app_closing:
                self.deleteLater()
            return
        except Exception as e:
            print("Whoops my except statement in motor missed the error that I was trying to catch.")
            raise e
        return

    @pyqtSlot()
    def ch1_v(self):
        """
        method to read the voltage on channel 1 of the motor. Value returned and also signals emitted.
        """
        cmd = self.ch1 + 'R?'
        rep = re.findall("\d+\.\d+", self.inst.query(cmd))
        try:
            self._ch1_v = [float(rep[0])]
            # Successful read.
            self.returning_ch1V_signal.emit(self.motor_number, 1, self._ch1_v[0])
        except IndexError:
            # Failed read.
            print("Warning: The ch1,", self.ch1, ", voltage on motor, ", self.motor_number, ", was not read correctly.")
            self.get_ch1V_failed_signal.emit(self.motor_number, 1)
        return self._ch1_v[0]

    @pyqtSlot()
    def ch2_v(self):
        """
        method to read the voltage on channel 1 of the motor. Value returned and also signals emitted.
        """
        cmd = self.ch2 + 'R?'
        rep = re.findall("\d+\.\d+", self.inst.query(cmd))
        try:
            self._ch2_v = [float(rep[0])]
            # Successful read.
            self.returning_ch2V_signal.emit(self.motor_number, 2, self._ch2_v[0])
        except IndexError:
            # Failed read.
            print("Warning: The ch2,", self.ch2, ", voltage on motor, ", self.motor_number, ", was not read correctly.")
            self.get_ch2V_failed_signal.emit(self.motor_number, 2)
        return self._ch2_v[0]

    @pyqtSlot()
    def ch3_v(self):
        """
        method to read the voltage on channel 3 of the motor. Value returned and also signals emitted.
        """
        cmd = self.ch3 + 'R?'
        rep = re.findall("\d+\.\d+", self.inst.query(cmd))
        try:
            self._ch3_v = [float(rep[0])]
            # Successful read.
            self.returning_ch3V_signal.emit(self.motor_number, 3, self._ch3_v[0])
        except IndexError:
            # Failed read.
            print("Warning: The ch3,", self.ch3, ", voltage on motor, ", self.motor_number, ", was not read correctly.")
            self.get_ch3V_failed_signal.emit(self.motor_number, 3)
        return self._ch3_v[0]

    @pyqtSlot(float)
    def set_ch1_v(self, v: float):
        """
        Set the voltage on motor channel number 1 to v. Emit signal to inform success or failure and set value.
        """
        cmd = self.ch1 + 'V' + str('{:.1f}'.format(v))
        rep = self.inst.query(cmd)
        rep = re.findall("\d+\.\d+", rep)  # Extract set Voltage from the full returned string
        if abs(float('{:.1f}'.format(v)) - float(rep[-1])) > 1.0:
            print("Warning: The ch1,", self.ch1, ", voltage on motor, ", self.motor_number, ", was not SET correctly. "
                                                                                            "attempted to set ", v)
            self.set_ch1V_failed_signal.emit(self.motor_number, 1)
        else:
            self.set_ch1V_complete_signal.emit(self.motor_number, 1, float('{:.1f}'.format(v)))
        return

    @pyqtSlot(float)
    def set_ch2_v(self, v: float):
        """
        Set the voltage on motor channel number 2 to v. Emit signal to inform success or failure and set value.
        """
        cmd = self.ch2 + 'V' + str('{:.1f}'.format(v))
        rep = self.inst.query(cmd)
        rep = re.findall("\d+\.\d+", rep)  # Extract set Voltage from the full returned string
        if abs(float('{:.1f}'.format(v)) - float(rep[-1])) > 1.0:
            print("Warning: The ch2,", self.ch2, ", voltage on motor, ", self.motor_number, ", was not SET correctly. "
                                                                                            "attempted to set ", v)
            self.set_ch2V_failed_signal.emit(self.motor_number, 2)
        else:
            self.set_ch2V_complete_signal.emit(self.motor_number, 2, float('{:.1f}'.format(v)))
        return

    @pyqtSlot(float)
    def set_ch3_v(self, v: float):
        """
        Set the voltage on motor channel number 3 to v. Emit signal to inform success or failure and set value.
        """
        cmd = self.ch3 + 'V' + str('{:.1f}'.format(v))
        rep = self.inst.query(cmd)
        rep = re.findall("\d+\.\d+", rep)  # Extract set Voltage from the full returned string
        if abs(float('{:.1f}'.format(v)) - float(rep[-1])) > 1.0:
            print("Warning: The ch3,", self.ch3, ", voltage on motor, ", self.motor_number, ", was not SET correctly. "
                                                                                            "attempted to set ", v)
            self.set_ch3V_failed_signal.emit(self.motor_number, 3)
        else:
            self.set_ch3V_complete_signal.emit(self.motor_number, 3, float('{:.1f}'.format(v)))
        return

    def get_info(self):
        cmd = 'I'
        rep = self.inst.query(cmd)
        return rep