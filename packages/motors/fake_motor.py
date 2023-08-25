import pyvisa as visa
from typing import NewType
from abc import ABC, abstractmethod

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

