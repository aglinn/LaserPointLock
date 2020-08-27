"""
Created by Matthew Teta

This is the main executable file. 
This file will instantiate CameraManager, MotorManager, and App
Passing the CameraManager and MotorManager instances into App will allow everything to communicate
"""

import sys
import time
import numpy as np
import visa
from PyQt5.QtCore import QStateMachine, QTimer

# local files
# from camera import MightexCamera, MightexEngine, DeviceNotFoundError, FakeCamera
from camera import FakeCamera
from motors import MDT693A_Motor, FakeMotor

from App import App

def mainExec():
    print("Attempting to create a window")
    app = App()



if __name__ == "__main__":
    mainExec()