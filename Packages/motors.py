import visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from typing import NewType
import re

VisaResourceManager = NewType('VisaResourceManager', visa.ResourceManager)

class FakeMotor():
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

class MDT693A_Motor():
    """
    Code that implements a Motor assuming a connection to the ThorLabs MDT693A
    """
    def __init__(self, rm: VisaResourceManager, com_port: str, ch1: str, ch2: str):
        """
        rm is a pyvisa ResourceManager
        com_port is the RS232 com port for the motor
        ch1 should be a string that helps direct the motor to control the software X
        ch2 should be a string that helps direct the motor to control the software Y
        """
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

    def terminate(self):
        # Close connection to port
        pass

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
        return
    
    def get_info(self):
        cmd = 'I'
        rep = self.inst.query(cmd)
        return rep
