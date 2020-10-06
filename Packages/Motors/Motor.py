

class Motor():
    kind = "Motor"

    def __init__(self):
        self.serial_no = ""

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


