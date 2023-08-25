from Thorlabs_MDT69XB_PythonSDK import MDT_COMMAND_LIB as mdt
import numpy as np
from packages.motors.base import BaseMotor


class MDT693BMotor(BaseMotor):

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

        # Init the BaseMotor Class.
        super().__init__(motor_number, ch1, ch2, ch3)
        # Now assign the properties the actual starting values.
        self._ch1_v = [self.ch1_v]
        self._ch2_v = [self.ch2_v]
        self._ch3_v = [self.ch3_v]
        # Can only query get_info once base class is instantiated in this case.
        self._id = []
        self.get_info(self._id)
        return