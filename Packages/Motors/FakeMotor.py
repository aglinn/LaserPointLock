

class FakeMotor(Motor):
    kind = "FakeMotor"
    voltageRange = (0.0, 100.0)

    def __init__(self, serial_no):
        self.serial_no = serial_no
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

