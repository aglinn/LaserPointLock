import visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from typing import NewType, List
import numpy

from Packages.Motors.Motor import Motor
from Packages.Motors.MDT693AMotor import MDT693AMotor
from Packages.Motors.FakeMotor import FakeMotor

class MotorManager():
    resourceManager: visa.ResourceManager = None

    def __init__(self):
        # Create resource manager to talk to devices
        self.resourceManager = visa.resourceManager()

        self.deviceList: List[Motor] = [] # All connected devices (String identifier)
        self.selectedDevices: List[int] = [] # Indecies of selected devices

    # returns list of all connected devices
    def getDeviceList(self):
        """
        This will be run upon launch of the app.
        """
        # Append two fake motors first, then real resouces from pyvisa
        self.deviceList = []
        self.deviceList.append(FakeMotor(1))
        self.deviceList.append(FakeMotor(2))
        self.deviceList.append(self.resourceManager.list_resources())
        return self.deviceList
    
    def getMotorList(self):
        return self.motorList

    def getMotor(self, index):
        return self.motorList[index]

    # Returns None if port is already in use
    def selectCamera(self, index: int):
        # All cameras in the list must be unique
        if not self.selectedDevices.index(index):
            self.selectedDevices.append(index)
            return index
        else:
            return -1
    
    # Returns mootor object given index of device index stored in selected devices
    def getSelectedMotor(self, index: int):
        """
        When the user chooses the motors to use in the GUI selection box, the GUI should inject a state into the
        statemachine queue. This state will prompt the state machine to trigger this function, which will establish the
        list of motors to be used in the ApplyVoltages method.
        """
        if index >=0 and index < len(self.selectedDevices):
            return self.deviceList[index]
        else:
            return None


    def terminate(self):
        # Terminate all motor devices
        for device in self.deviceList:
            device.terminate()

    def ApplyVoltages(self, Voltages: list):
        """
        This function is run by the state manager.

        Purpose: set voltages on the motors in the list.

        input: 4 component list of voltages=[motor1_x, motor1_y, motor2_x, motor2_y]

        return: nothing.

        You need to loop over your motorlist returned from getSelectedMotors and run the setXVoltage and setYVoltage
        methods of each motor with the appropriate voltage from the Voltages list.
        """

    def ReportVoltages(self):
        """
        At some point we need to get information about the current voltages into the UpdateManager; so that object can
        find the new voltages to be applied. We can either assume voltages are always the previous set value, as in we
        never need to read the voltages from the machines, although we know sometimes setting the voltages failed.
        """

    # # Returns None if port is already in use
    # def addMotor(self, port: str, kind: str):
    #     if self.portList.index(port):
    #         return None
        
    #     m: Motor = None
    #     if kind == "MDT693AMotor":
    #         # Open port to communicate with motor
    #         mInst = self.resourceManager.open_resource(port, baud_rate=MDT693AMotor.baud_rate, data_bits=MDT693AMotor.data_bits, parity=MDT693AMotor.parity, flow_control=MDT693AMotor.flow_control, stop_bits=MDT693AMotor.stop_bits)
    #         m = MDT693AMotor(mInst)
    #         self.portList.append(port)
    #     else:
    #         m = FakeMotor()

    #     self.motorList.append(m)
    #     return m

    # def removeMotor(self, index):
    #     if index >= 0 and index < len(self.motorList):
    #         m = self.motorList[index]
    #         self.motorList.remove(m)
    #         if m.kind == "MDT693AMotor":
    #             m.inst.close()

    # def removeAllMotors(self):
    #     for i in range(len(self.motorList)):
    #         self.removeMotor(i)
