"""
Created by Matthew Teta

This class is responsible for handling interactions between the GUI, the cameras, and the motors
"""

import sys
from PyQt5.QtWidgets import QApplication, QErrorMessage

from MainWindow import MainWindow

class App:

    def __init__(self):
        self.app = QApplication(sys.argv)
        self.GUI = MainWindow()
        sys.exit(self.app.exec_())
        