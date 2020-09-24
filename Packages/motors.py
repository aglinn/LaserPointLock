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


class Motor():
    kind = "Motor"

    def getVoltageRange(self):
        return [0.0, 100.0]

    def getXVoltage(self):
        pass

    def getYVoltage(self):
        pass

    def getVoltages(self):
        pass

    def setXVoltage(self, v: float):
        pass

    def setYVoltage(self, v: float):
        pass

    def setVoltages(self, v: tuple):
        pass


class FakeMotor(Motor):
    kind = "FakeMotor"
    voltageRange = (0.0, 100.0)

    def __init__(self):
        self.xv = 0 # X Voltage
        self.yv = 0 # Y Voltage
        self.xcount = 2
        self.ycount = 2

    def getVoltageRange(self):
        return self.voltageRange

    def getXVoltage(self):
        r = self.xv / self.count
        self.xcount -= 1
        if self.xcount == 0:
            self.xcount = 2
        return r

    def getYVoltage(self):
        r = self.yv / self.count
        self.ycount -= 1
        if self.ycount == 0:
            self.ycount = 2
        return r

    def getVoltages(self):
        return (self.getXVoltage(), self.getYVoltage())

    def setXVoltage(self, v: float):
        if v < self.voltageRange[0]:
            self.xv = self.voltageRange[0]
        elif v > self.voltageRange[1]:
            self.xv = self.voltageRange[1]
        else:
            self.xv = v
        return self.xv

    def setYVoltage(self, v: float):
        if v < self.voltageRange[0]:
            self.yv = self.voltageRange[0]
        elif v > self.voltageRange[1]:
            self.yv = self.voltageRange[1]
        else:
            self.yv = v
        return self.yv

    def setVoltages(self, v: tuple):
        pass

class MDT693AMotor(Motor):
    """
    Code that implements a Motor assuming a connection to the ThorLabs MDT693A

    x_id should be a string that helps direct the motor to control the software X
    y_id should be a string that helps direct the motor to control the software Y

    # Baud Rate: 115200
    # Date bits: 8
    # Parity: none
    # Stop Bits: 1
    # Flow control: none
    """

    xId = 'X'
    yId = 'Y'

    baud_rate=115200
    data_bits=8
    parity=Parity.none
    flow_control=VI_ASRL_FLOW_NONE
    stop_bits=StopBits.one

    voltageRange = (0.0, 100.0)

    def __init__(self, resourceInst):
        super().__init__()
        
        self.inst = resourceInst
        self.xv = 75.0
        self.yv = 75.0
        self.setXVoltage(self.xv)
        self.setYVoltage(self.yv)

    def getXVoltage(self):
        cmd = f'{self.xID} R?'
        while attempt in range(10): # Try to get value at most 10 times before raising an error
            try:
                reading = float(re.findall("\d+\.\d+", self.inst.query(cmd))[0])
                # Return the last setpoint of the voltage, unless the readout is more than 0.3V away,
                # because I found that the readout can be wrong by 0.2
                if reading:
                    if abs(reading - self.xv) > 0.3:
                        return self.xv
                    return reading
            except IndexError: # No match regex
                print("Trying to read motor voltage again.")
        print("Could not read motor voltage")
        return None

    def getYVoltage(self):
        cmd = f'{self.yID} R?'
        while attempt in range(10): # Try to get value at most 10 times before raising an error
            try:
                reading = float(re.findall("\d+\.\d+", self.inst.query(cmd))[0])
                # Return the last setpoint of the voltage, unless the readout is more than 0.3V away,
                # because I found that the readout can be wrong by 0.2
                if reading:
                    if abs(reading - self.yv) > 0.3:
                        return self.yv
                    return reading
            except IndexError: # No match regex
                print("Trying to read motor voltage again.")
        print("Could not read motor voltage")
        return None

    def getVoltages(self):
        return (self.getXVoltage(), self.getYVoltage())

    def setXVoltage(self, v: float):
        if v < self.voltageRange[0]:
            self.xv = self.voltageRange[0]
        elif v > self.voltageRange[1]:
            self.xv = self.voltageRange[1]
        else:
            self.xv = v

        cmd = f'{self.xId}V' + str('{:.1f}'.format(self.xv))
        while attempt in range(10):
            try:
                # Set voltage by sending cmd
                rep = self.inst.query(cmd)
                # Extract set Voltage from the full returned string
                reading = self.getXVoltage()
                if reading:
                    if abs(reading - self.xv) > 0.5:
                        raise NotImplementedError('Could not set X voltage correctly.')
                    else:
                        return reading
            except IndexError:
                print("Trying to read motor voltage again.")
        print("Could not read motor voltage, X motor voltage is likely set incorrectly")
        return None

    def setYVoltage(self, v: float):
        if v < self.voltageRange[0]:
            self.yv = self.voltageRange[0]
        elif v > self.voltageRange[1]:
            self.yv = self.voltageRange[1]
        else:
            self.yv = v

        cmd = f'{self.yId}V' + str('{:.1f}'.format(self.yv))
        while attempt in range(10):
            try:
                # Set voltage by sending cmd
                rep = self.inst.query(cmd)
                # Extract set Voltage from the full returned string
                reading = self.getXVoltage()
                if reading:
                    if abs(reading - self.yv) > 0.5:
                        raise NotImplementedError('Could not set Y voltage correctly.')
                    else:
                        return reading
            except IndexError:
                print("Trying to read motor voltage again.")
        print("Could not read motor voltage, Y motor voltage is likely set incorrectly")

        return None

    def setVoltages(self, v: tuple):
        return (self.setXVoltage(v[0]), self.setYVoltage(v[1]))
    
    def getInfo(self):
        cmd = 'I'
        rep = self.inst.query(cmd)
        return rep
