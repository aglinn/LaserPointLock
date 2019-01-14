import visa
from pyvisa.constants import StopBits, Parity, VI_ASRL_FLOW_NONE
from typing import NewType

VisaResourceManager = NewType('VisaResourceManager', visa.ResourceManager)

class MDT693A_Motor():
    """
    Code that implements a Motor assuming a connection to the ThorLabs MDT693A
    """
    def __init__(self, rm: VisaResourceManager, com_port: str, ch1: str, ch2: str):
        """
        rm is a pyvisa ResourceManager
        com_port is the RS232 com port for the motor
        ch1 is a string 'X' or 'Y' (no error checking for 'Z')
        ch2 is a string 'X' or 'Y'
        """
        # Baud Rate: 115200
        # Date bits: 8
        # Parity: none
        # Stop Bits: 1
        # Flow control: none
        self.port = com_port
        self.inst = rm.open_resource(com_port, baud_rate=115200, data_bits=8, parity=Parity.none, flow_control=VI_ASRL_FLOW_NONE, stop_bits=StopBits.one)
        self.ch1 = ch1
        self.ch2 = ch2

    def terminate(self):
        # Close connection to port
        pass

    def change_port(self, val):
        # Close connection to port
        # Open connection to new port
        pass

    @property
    def ch1(self):
        return self.ch1

    @property
    def ch2(self):
        return self.ch2

    @ch1.setter
    def ch1(self, val: float):
        self.ch1 = val
    
    @ch2.setter
    def ch2(self, val: float):
        self.ch2 = val

    @property
    def ch1_v(self):
        cmd = self.ch1 + 'R?'
        rep = self.inst.query(cmd)
        return rep
    
    @property
    def ch2_v(self):
        cmd = self.ch2 + 'R?'
        rep = self.inst.query(cmd)
        return rep

    @ch1_v.setter
    def ch1_v(self, v: float):
        self.ch1 = v
        cmd = self.ch1 + 'V ' + '{:.1f}'.format(v)
        rep = self.inst.query(cmd)
        return rep

    @ch2_v.setter
    def ch2_v(self, v: float):
        self.ch2 = v
        cmd = self.ch2 + 'V ' + '{:.1f}'.format(v)
        rep = self.inst.query(cmd)
        return rep
    
    def get_info(self):
        cmd = 'I'
        rep = self.inst.query(cmd)
        return rep
