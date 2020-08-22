if __name__ == "__main__":
    import sys
    import visa
    import time
    import numpy as np
    import pyqtgraph as pg
    from pointing_ui import Ui_MainWindow
    from PyQt5 import QtCore, QtGui, QtWidgets,QtSvg
    from camera import MightexCamera, MightexEngine, DeviceNotFoundError, FakeCamera
    from motors import MDT693A_Motor, FakeMotor
    import tkinter as tk
    from tkinter import filedialog

    print("test")