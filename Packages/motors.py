import visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from typing import NewType
import re
import time
from Thorlabs_MDT69XB_PythonSDK import MDT_COMMAND_LIB as mdt
import numpy as np
from abc import ABC,abstractmethod
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import ctypes

VisaResourceManager = NewType('VisaResourceManager', visa.ResourceManager)

class Motor(ABC):

    @abstractmethod
    def close(self):
        pass
        return

    @property
    @abstractmethod
    def ch1(self):
        pass
        return

    @property
    @abstractmethod
    def ch2(self):
        pass
        return

    @ch1.setter
    @abstractmethod
    def ch1(self, val: str):
        pass
        return

    @ch2.setter
    @abstractmethod
    def ch2(self, val: str):
        pass
        return

    @property
    @abstractmethod
    def ch1_v(self):
        pass
        return

    @property
    @abstractmethod
    def ch2_v(self):
        pass
        return

    @ch1_v.setter
    @abstractmethod
    def ch1_v(self, v: float):
        pass
        return

    @ch2_v.setter
    @abstractmethod
    def ch2_v(self, v: float):
        pass
        return

    @abstractmethod
    def get_info(self, id):
        pass
        return

class FakeMotor(Motor):
    def __init__(self, ch1: str, ch2: str):
        self._ch1 = ch1
        self._ch2 = ch2
        self._count = 2  # the motor will require multiple sets to reach its target
        self._ch1_v = 0
        self._ch2_v = 0

    @property
    def ch1(self):
        return self._ch1

    @property
    def ch2(self):
        return self._ch2

    @ch1.setter
    def ch1(self, val: str):
        self._ch1 = val
    
    @ch2.setter
    def ch2(self, val: str):
        self._ch2 = val

    @property
    def ch1_v(self):
        if self._count <= 0:
            self._count = 2
            # print('Read', self._ch1_v, 'on ch 1')
            return self._ch1_v
        else:
            self._count -= 1
            # print('Read', self._ch1_v / (self._count + 1), 'on ch 1')
            return self._ch1_v / (self._count + 1)
    
    @property
    def ch2_v(self):
        if self._count <= 0:
            self._count = 2
            # print('Read', self._ch2_v, 'on ch 2')
            return self._ch2_v
        else:
            self._count -= 1
            # print('Read', self._ch2_v / (self._count + 1), 'on ch 2')
            return self._ch2_v / (self._count + 1)

    @ch1_v.setter
    def ch1_v(self, v: float):
        # print('Setting ch 1 voltage to', v)
        self._ch1_v = v

    @ch2_v.setter
    def ch2_v(self, v: float):
        # print('Setting ch 2 voltage to', v)
        self._ch2_v = v

class MDT693A_Motor(Motor):
    """
    Code that implements a Motor assuming a connection to the ThorLabs MDT693A
    """
    def __init__(self, rm: VisaResourceManager, motor_number, com_port: str, ch1: str, ch2: str):
        """
        rm is a pyvisa ResourceManager
        com_port is the RS232 com port for the motor
        ch1 should be a string that helps direct the motor to control the software X
        ch2 should be a string that helps direct the motor to control the software Y
        """
        #Assign what motor number this motor is.
        self.motor_number = motor_number
        # Baud Rate: 115200
        # Date bits: 8
        # Parity: none
        # Stop Bits: 1
        # Flow control: none
        self.port = com_port
        self.inst = rm.open_resource(com_port, baud_rate=115200, data_bits=8, parity=Parity.none, flow_control=VI_ASRL_FLOW_NONE, stop_bits=StopBits.one)
        self._ch1 = ch1
        self._ch2 = ch2
        self.inst.query('XV75.0')
        self.inst.query('YV75.0')
        self._ch1_v = 75.0
        self._ch2_v = 75.0

    def close(self):
        # Close connection to port
        self.inst.close()
        return

    def change_port(self, val):
        # Close connection to port
        # Open connection to new port
        pass

    @property
    def ch1(self):
        return self._ch1

    @property
    def ch2(self):
        return self._ch2

    @ch1.setter
    def ch1(self, val: str):
        self._ch1 = val
    
    @ch2.setter
    def ch2(self, val: str):
        self._ch2 = val

    @property
    def ch1_v(self):
        """cmd = self.ch1 + 'R?'
        TryAgain=True
        count = 0
        while TryAgain == True:
            rep = re.findall("\d+\.\d+", self.inst.query(cmd))
            # Return the last setpoint of the voltage, unless the readout is more than 0.3V away, because I found that the readout can be wrong by 0.2
            try:
                if abs(float(rep[0])-self._ch1_v)>0.3:
                    return float(rep[0])
                else:
                    return self._ch1_v
                TryAgain = False
            except IndexError:
                count += 1
            if count==1:
                print("Shit, never could read the ch1_v, giving up")
                TryAgain = False"""
        return self._ch1_v

    @property
    def ch2_v(self):
        """cmd = self.ch2 + 'R?'
        #Return the last setpoint of the voltage, unless the readout is more than 0.3V away, because I found that the readout can be wrong by 0.2
        TryAgain = True
        count = 0
        while TryAgain == True:
            rep = re.findall("\d+\.\d+", self.inst.query(cmd))
            # Return the last setpoint of the voltage, unless the readout is more than 0.3V away, because I found that the readout can be wrong by 0.2
            try:
                if abs(float(rep[0]) - self._ch2_v) > 0.3:
                    return float(rep[0])
                else:
                    return self._ch2_v
                TryAgain = False
            except IndexError:
                count += 1
            if count == 1:
                print("Shit, never could read the ch2_v, giving up")
                TryAgain = False"""
        return self._ch2_v

    @ch1_v.setter
    def ch1_v(self, v: float):
        self._ch1_v = v
        cmd = self.ch1 + 'V' + str('{:.1f}'.format(v))
        rep = self.inst.query(cmd)
        rep = re.findall("\d+\.\d+", rep) # Extract set Voltage from the full returned string
        if abs(float('{:.1f}'.format(v))-float(rep[-1])) > 1.0:
            print('Shit the Ch1 Voltage was never set correctly.')
            cmd = self.ch1 + 'R?'
            rep = re.findall("\d+\.\d+", self.inst.query(cmd))
        self._ch1_v = float(rep[-1])
        #time.sleep(1/4)
        return

    @ch2_v.setter
    def ch2_v(self, v: float):
        self._ch2_v = v
        cmd = self.ch2 + 'V' + str('{:.1f}'.format(v))
        rep = self.inst.query(cmd)
        rep = re.findall("\d+\.\d+", rep)  # Extract set Voltage from the full returned string
        if abs(float('{:.1f}'.format(v)) - float(rep[-1])) > 1.0:
            print('Shit the Ch2 Voltage was never set correctly.')
            cmd = self.ch2 + 'R?'
            rep = re.findall("\d+\.\d+", self.inst.query(cmd))
        self._ch2_v = float(rep[-1])
        #time.sleep(1/4)
        return
    
    def get_info(self):
        cmd = 'I'
        rep = self.inst.query(cmd)
        return rep

class MDT693B_Motor(QObject):
    """
    Code that implements a Motor assuming a connection to the ThorLabs MDT693B
    using Thorlabs SDK. Please reimplement all methods in Motor class, eventhough it is not explicitly a parent of this
    class.

    Subclass QObject. Create signals and connect them to slots.
    """
    # first int on any emitted signals should be motor number.
    # If there is a second int, it indicates the channel number.
    # Motor Signals
    request_close_signal = pyqtSignal()
    close_complete_signal = pyqtSignal(int)
    close_fail_signal = pyqtSignal(int)
    #Ch1 signals
    request_set_ch1V_signal = pyqtSignal(float)
    set_ch1V_complete_signal = pyqtSignal(int, int, float)
    set_ch1V_failed_signal = pyqtSignal(int, int)
    request_get_ch1V_signal = pyqtSignal()
    get_ch1V_failed_signal = pyqtSignal(int, int)
    returning_ch1V_signal = pyqtSignal(int, int, float)
    #Ch2 signals
    request_set_ch2V_signal = pyqtSignal(float)
    set_ch2V_complete_signal = pyqtSignal(int, int, float)
    set_ch2V_failed_signal = pyqtSignal(int, int)
    request_get_ch2V_signal = pyqtSignal()
    get_ch2V_failed_signal =pyqtSignal(int, int)
    returning_ch2V_signal = pyqtSignal(int, int, float)

    def __init__(self, serial_number, motor_number, ch1: str = 'X', ch2: str = 'Y'):
        """This constructor opens the motor, sets the appropriate voltage bounds, and sets the voltage increment."""
        super().__init__()
        """Open the motor"""
        # Assign which motor number this motor is:
        self.motor_number = motor_number
        # Baud rate is 115200 bits/s
        # set 1 second timeout
        self.handle = mdt.mdtOpen(serial_number, 115200, 1)
        if self.handle < 0:
            raise Exception("Motor with serial number,", serial_number, " did not open correctly.")
        self._serial_number = serial_number
        self._id = []
        self.get_info(self._id)

        """Now make sure that the voltage step resolution is as small as possible"""
        ret = mdt.mdtSetVoltageAdjustmentResolution(self.handle, 1)
        if ret < 0:
            raise Exception("The adjustment resolution on Motor with serial number,", serial_number,
                            " did not set correctly.")
        self._voltage_step_resolution = np.array([0])
        ret = mdt.mdtGetVoltageAdjustmentResolution(self.handle, self._voltage_step_resolution)
        if ret < 0:
            raise Exception("The adjustment resolution on Motor with serial number,", serial_number,
                            " did not read correctly.")

        """Disable masterscan mode"""
        ret = mdt.mdtSetMasterScanEnable(self.handle, 0)
        if ret < 0:
            raise Exception("Master Scan could not be disabled")
        self._master_scan_state = np.array([-2])
        ret = mdt.mdtGetMasterScanEnable(self.handle, self._master_scan_state)
        if ret < 0:
            raise Exception("Could not read the master scan state.")
        if self._master_scan_state[0] != 0:
            raise Exception("Master scan state should be disabled, but it is not?")


        """Set the correct min and max voltages on X and Y channels of motor"""
        #x axis
        self._x_max_voltage = np.array([0])
        ret = mdt.mdtGetXAxisMaxVoltage(self.handle, self._x_max_voltage)
        if ret < 0:
            raise Exception("Failed to read x-axis max voltage")
        if not self._x_max_voltage[0] == 150:
            ret = mdt.mdtSetXAxisMaxVoltage(self.handle, 150.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set Xmax voltage correctly.")
            ret = mdt.mdtGetXAxisMaxVoltage(self.handle, self._x_max_voltage)
            if ret < 0:
                raise Exception("Failed to read x-axis max voltage")
            print("Motor with serial number,", serial_number, "x axis max voltage set to:", self._x_max_voltage)
        self._x_min_voltage = np.array([0])
        ret = mdt.mdtGetXAxisMinVoltage(self.handle, self._x_min_voltage)
        if ret < 0:
            raise Exception("Failed to read x-axis min voltage")
        if not self._x_min_voltage[0] == 0:
            ret = mdt.mdtSetXAxisMinVoltage(self.handle, 0.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set Xmin voltage correctly.")
            ret = mdt.mdtGetXAxisMinVoltage(self.handle, self._x_min_voltage)
            if ret < 0:
                raise Exception("Failed to read x-axis min voltage")
            print("Motor with serial number,", serial_number, "x axis min voltage set to:", self._x_min_voltage)
        # repeat for y axis
        self._y_max_voltage = np.array([0])
        ret = mdt.mdtGetYAxisMaxVoltage(self.handle, self._y_max_voltage)
        if ret < 0:
            raise Exception("Failed to read y-axis max voltage")
        if not self._y_max_voltage[0] == 150:
            ret = mdt.mdtSetYAxisMaxVoltage(self.handle, 150.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set Ymax voltage correctly.")
            ret = mdt.mdtGetYAxisMaxVoltage(self.handle, self._y_max_voltage)
            if ret < 0:
                raise Exception("Failed to read y-axis max voltage")
            print("Motor with serial number,", serial_number, "y axis max voltage set to:", self._y_max_voltage)
        self._y_min_voltage = np.array([0])
        ret = mdt.mdtGetYAxisMinVoltage(self.handle, self._y_min_voltage)
        if ret < 0:
            raise Exception("Failed to read y-axis min voltage")
        if not self._y_min_voltage[0] == 0:
            ret = mdt.mdtSetYAxisMinVoltage(self.handle, 0.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set Ymin voltage correctly.")
            ret = mdt.mdtGetYAxisMinVoltage(self.handle, self._y_min_voltage)
            if ret < 0:
                raise Exception("Failed to read y-axis min voltage")
            print("Motor with serial number,", serial_number, "y axis min voltage set to:", self._y_min_voltage)

        self._ch1 = ch1
        self._ch2 = ch2
        self._ch1_v = ctypes.c_double(0)
        self._ch2_v = ctypes.c_double(0)
        self._ch1_v = self.ch1_v
        self._ch2_v = self.ch2_v
        self.connect_signals()
        self._app_closing = False
        return

    def connect_signals(self):
        self.request_close_signal.connect(self.close)
        self.request_set_ch1V_signal.connect(lambda v: setattr(MDT693B_Motor, 'ch1_v', v))
        self.request_get_ch1V_signal.connect(self.ch1_v)
        self.request_set_ch2V_signal.connect(lambda v: setattr(MDT693B_Motor, 'ch2_v', v))
        self.request_get_ch2V_signal.connect(self.ch2_v)
        return

    @pyqtSlot()
    def close(self):
        # Close connection to port
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

    def __del__(self):
        print('destructor called for motor number, ', self.motor_number)
        return

    @property
    def ch1(self):
        return self._ch1

    @property
    def ch2(self):
        return self._ch2

    @ch1.setter
    def ch1(self, val: str):
        self._ch1 = val

    @ch2.setter
    def ch2(self, val: str):
        self._ch2 = val

    @property
    def ch1_v(self):
        if self.ch1 == 'X' or self.ch1 == 'x':
            ret = mdt.mdtGetXAxisVoltage(self.handle, ctypes.byref(self._ch1_v))
        elif self.ch1 == 'Y' or self.ch1 == 'y':
            ret = mdt.mdtGetYAxisVoltage(self.handle, ctypes.byref(self._ch1_v))
        if ret < 0:
            print("Warning: The ch1,", self.ch1, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not read correctly.")
            self.get_ch1V_failed_signal.emit(self.motor_number, 1)
        else:
            self.returning_ch1V_signal.emit(self.motor_number, 1, self._ch1_v)
        return self._ch1_v

    @property
    def ch2_v(self):
        if self.ch2 == 'X' or self.ch2 == 'x':
            ret = mdt.mdtGetXAxisVoltage(self.handle, ctypes.byref(self._ch2_v))
        elif self.ch2 == 'Y' or self.ch2 == 'y':
            ret = mdt.mdtGetYAxisVoltage(self.handle, ctypes.byref(self._ch2_v))
        if ret < 0:
            print("Warning: The voltage on ch2,", self.ch2, ", on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not read correctly.")
            self.get_ch2V_failed_signal.emit(self.motor_number, 2)
        else:
            self.returning_ch2V_signal.emit(self.motor_number, 2, self._ch2_v)
        return self._ch2_v

    @ch1_v.setter
    def ch1_v(self, v: float):
        v = np.round(v, 3)
        if self.ch1 == 'X' or self.ch1 == 'x':
            rep = mdt.mdtSetXAxisVoltage(self.handle, v)
        elif self.ch1 == 'Y' or self.ch1 == 'y':
            rep = mdt.mdtSetYAxisVoltage(self.handle, v)
        if rep < 0:
            print("Warning: The ch1,", self.ch1, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not SET correctly.")
            self.set_ch1V_failed_signal.emit(self.motor_number, 1)
        else:
            self.set_ch1V_complete_signal.emit(self.motor_number, 1, v)
        return

    @ch2_v.setter
    def ch2_v(self, v: float):
        v = np.round(v, 3)
        if self.ch2 == 'X' or self.ch2 == 'x':
            rep = mdt.mdtSetXAxisVoltage(self.handle, v)
        elif self.ch2 == 'Y' or self.ch2 == 'y':
            rep = mdt.mdtSetYAxisVoltage(self.handle, v)
        if rep < 0:
            print("Warning: The ch1,", self.ch2, ", voltage on motor with handle, ", self.handle,
                  ", and serial number,", self._serial_number, " was not SET correctly.")
            self.set_ch2V_failed_signal.emit(self.motor_number, 2)
        else:
            self.set_ch2V_complete_signal.emit(self.motor_number, 2, v)
        return

    def get_info(self, id):
        ret = mdt.mdtGetId(self.handle, id)
        if ret < 0:
            print("Warning: Could not get the id from the motor with serial number = ", self._serial_number)
        return
