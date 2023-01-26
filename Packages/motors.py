import visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from pyvisa.errors import InvalidSession
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


class Base_Motor(QObject):
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


# TODO: Test this new version of MDT693A
class MDT693AMotor(Base_Motor):
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


# TODO: Test this new code for MDT693B motor.
class MDT693BMotor(Base_Motor):

    def __init__(self, serial_number, motor_number, ch1: str = 'X', ch2: str = 'Y', ch3: str = 'Z'):
        """This constructor opens the motor, sets the appropriate voltage bounds, and sets the voltage increment."""
        #########################################################################################################
        # Perform extra init steps for MDT693B First, so that motor exists to query during init of parent class #
        #########################################################################################################
        """Open the motor"""
        # Baud rate is 115200 bits/s
        # set 1 second timeout
        self.handle = mdt.mdtOpen(serial_number, 115200, 1)
        if self.handle < 0:
            raise Exception("Motor with serial number,", serial_number, " did not open correctly.")
        self._serial_number = serial_number
        """
        I actually do not think that this does anything when controlling with my application.
        Now make sure that the voltage step resolution is as small as possible"""
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

        """Set the correct min and max voltages on X, Y, and Z channels of motor"""
        # x axis
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
        # repeat for z axis
        self._z_max_voltage = np.array([0])
        ret = mdt.mdtGetZAxisMaxVoltage(self.handle, self._z_max_voltage)
        if ret < 0:
            raise Exception("Failed to read z-axis max voltage")
        if not self._z_max_voltage[0] == 150:
            ret = mdt.mdtSetZAxisMaxVoltage(self.handle, 150.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set z max voltage correctly.")
            ret = mdt.mdtGetZAxisMaxVoltage(self.handle, self._z_max_voltage)
            if ret < 0:
                raise Exception("Failed to read z-axis max voltage")
            print("Motor with serial number,", serial_number, "z axis max voltage set to:", self._z_max_voltage)
        self._z_min_voltage = np.array([0])
        ret = mdt.mdtGetYAxisMinVoltage(self.handle, self._z_min_voltage)
        if ret < 0:
            raise Exception("Failed to read z-axis min voltage")
        if not self._z_min_voltage[0] == 0:
            ret = mdt.mdtSetZAxisMinVoltage(self.handle, 0.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set z min voltage correctly.")
            ret = mdt.mdtGetZAxisMinVoltage(self.handle, self._z_min_voltage)
            if ret < 0:
                raise Exception("Failed to read z-axis min voltage")
            print("Motor with serial number,", serial_number, "z axis min voltage set to:", self._z_min_voltage)

        # Init the Base_Motor Class.
        super().__init__(motor_number, ch1, ch2, ch3)
        # Now assign the properties the actual starting values.
        self._ch1_v = [self.ch1_v]
        self._ch2_v = [self.ch2_v]
        self._ch3_v = [self.ch3_v]
        # Can only query get_info once base class is instantiated in this case.
        self._id = []
        self.get_info(self._id)
        return


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

    def __init__(self, serial_number, motor_number, ch1: str = 'X', ch2: str = 'Y', ch3: str = 'Z'):
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
        """
        I actually do not think that this does anything when controlling with my application.
        Now make sure that the voltage step resolution is as small as possible"""
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
        # repeat for z axis
        self._z_max_voltage = np.array([0])
        ret = mdt.mdtGetZAxisMaxVoltage(self.handle, self._z_max_voltage)
        if ret < 0:
            raise Exception("Failed to read z-axis max voltage")
        if not self._z_max_voltage[0] == 150:
            ret = mdt.mdtSetZAxisMaxVoltage(self.handle, 150.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set z max voltage correctly.")
            ret = mdt.mdtGetZAxisMaxVoltage(self.handle, self._z_max_voltage)
            if ret < 0:
                raise Exception("Failed to read z-axis max voltage")
            print("Motor with serial number,", serial_number, "z axis max voltage set to:", self._z_max_voltage)
        self._z_min_voltage = np.array([0])
        ret = mdt.mdtGetYAxisMinVoltage(self.handle, self._z_min_voltage)
        if ret < 0:
            raise Exception("Failed to read z-axis min voltage")
        if not self._z_min_voltage[0] == 0:
            ret = mdt.mdtSetZAxisMinVoltage(self.handle, 0.0)
            if ret < 0:
                raise Exception("Motor with serial number,", serial_number, " did not set z min voltage correctly.")
            ret = mdt.mdtGetZAxisMinVoltage(self.handle, self._z_min_voltage)
            if ret < 0:
                raise Exception("Failed to read z-axis min voltage")
            print("Motor with serial number,", serial_number, "z axis min voltage set to:", self._z_min_voltage)

        self._ch1 = ch1
        self._ch2 = ch2
        self._ch3 = ch3
        self._ch1_v = [0]
        self._ch2_v = [0]
        self._ch3_v = [0]
        self._ch1_v = [self.ch1_v]
        self._ch2_v = [self.ch2_v]
        self._ch3_v = [self.ch3_v]
        self._app_closing = False
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
        v = np.round(v, 3)
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
        v = np.round(v, 3)
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
        v = np.round(v, 3)
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
        ret = mdt.mdtGetId(self.handle, id)
        if ret < 0:
            print("Warning: Could not get the id from the motor with serial number = ", self._serial_number)
        return
