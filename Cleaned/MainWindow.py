"""
Created by Matthew Teta

This class implements the main GUI window including the tab layout manager
"""


from PyQt5.QtCore import QRect, Qt
from PyQt5.QtGui import QStandardItemModel, QIcon
from PyQt5.QtWidgets import QMainWindow, QErrorMessage, QWidget, QTabWidget, QMenuBar, QMenu, QAction, QStatusBar
import pyqtgraph as pg

from pointing_ui import Ui_MainWindow

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()

        self.setWindowTitle("Laser Point Lock")
        self.resize(1072, 780) # Set default window size
        # self.setWindowIcon(QIcon('pythonlogo.png'))

        # self.ui = Ui_MainWindow()
        # self.ui.setupUi(self)
        # cam_model = QStandardItemModel()
        # motor_model = QStandardItemModel()
        # error_dialog = QErrorMessage()

        self.setupMenuBar()
        self.setupTabLayout()
        self.show()

    def setupMenuBar(self):
        # Create menuBar
        self.menuBar = QMenuBar(self)
        self.menuBar.setGeometry(QRect(0, 0, 1072, 21))
        self.menuBar.setObjectName("MainWindow_menuBar")

        # Create "File" menu
        self.fileMenu = QMenu(self.menuBar)
        self.fileMenu.setObjectName("MainWindow_fileMenu")
        self.fileMenu.setTitle("File")

        # Create two menu items (Calibrate and Load Old Home)
        self.actionCalibrate = QAction(self)
        self.actionCalibrate.setObjectName("MainWindow_actionCalibrate")
        self.actionCalibrate.setText("Calibrate")
        self.actionLoadOldHome = QAction(self)
        self.actionLoadOldHome.setObjectName("MainWindow_actionLoadOldHome")
        self.actionLoadOldHome.setText("Load Stored Home")

        # Add menu items to file menu
        self.fileMenu.addAction(self.actionCalibrate)
        self.fileMenu.addAction(self.actionLoadOldHome)

        # Add file menu to menu bar
        self.menuBar.addMenu(self.fileMenu)
        self.menuBar.addAction(self.fileMenu.menuAction())

        self.setMenuBar(self.menuBar)

    def setupTabLayout(self):
        # Create bounding box in window
        self.bodyWidget = QWidget(self)
        self.bodyWidget.setObjectName("MainWindow_bodyWidget")
        self.setCentralWidget(self.bodyWidget)

        # Create tab widget inside of bodyWidget
        self.tabWidget = QTabWidget(self.bodyWidget)
        self.tabWidget.setGeometry(QRect(20, 20, 981, 631))
        self.tabWidget.setTabPosition(QTabWidget.North)
        self.tabWidget.setTabShape(QTabWidget.Rounded)
        self.tabWidget.setElideMode(Qt.ElideNone)
        self.tabWidget.setObjectName("MainWindow_tabWidget")
