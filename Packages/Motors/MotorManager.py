import visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from typing import NewType
import re
import numpy

class MotorManager():
    resourceManager: visa.ResourceManager = None

    def __init__(self):
        # Create resource manager to talk to devices
        self.resourceManager = visa.resourceManager()

        self.portList = []
        self.motorList = []

    # returns list of all connected devices
    def getDevices(self):
        return self.resourceManager.list_resources()
    
    def getMotorList(self):
        return self.motorList

    def getMotor(self, index):
        return self.motorList[index]

    # Returns None if port is already in use
    def addMotor(self, port: str, kind: str):
        if self.portList.index(port):
            return None
        
        m: Motor = None
        if kind == "MDT693AMotor":
            # Open port to communicate with motor
            mInst = self.resourceManager.open_resource(port, baud_rate=MDT693AMotor.baud_rate, data_bits=MDT693AMotor.data_bits, parity=MDT693AMotor.parity, flow_control=MDT693AMotor.flow_control, stop_bits=MDT693AMotor.stop_bits)
            m = MDT693AMotor(mInst)
            self.portList.append(port)
        else:
            m = FakeMotor()

        self.motorList.append(m)
        return m

    def removeMotor(self, index):
        if index >= 0 and index < len(self.motorList):
            m = self.motorList[index]
            self.motorList.remove(m)
            if m.kind == "MDT693AMotor":
                m.inst.close()

    def removeAllMotors(self):
        for i in range(len(self.motorList)):
            self.removeMotor(i)
