# TODO: 2 stop using global vars and put this code into a fucking window
# TODO: 1 Add statistics logging.
# TODO: 1 Let me FFT the camera COM coordinates to find out which frequencies are generating the noise.
# TODO: 1 For the above two, it may be useful to have a new state or even a seperate program to run diagnostics in a super
#  lean fashion; so we can be sensitive to the highest possible frequencies.
# TODO: 1 Finish overhauling code to allow for operation with IR Cameras.
# TODO: 3 Try using OpenCV with the Mightex cameras for faster operation
# TODO: 2 Try to shutter the TOPAS if the BOSON FPAs get too hot. Can I even control the TOPAS shutter?
# TODO: 2 Stupid bug fix: When I reduce the number of points to be plotted for COM and piezzo values, I need to make the
#  number of points plotted actually smaller.
# TODO: 2 try the other algorithm from the paper I found.
# TODO: 2 Implement PID, instead of just D.
# TODO: 2 Make the GUI compatible with multiple screen formats.
# TODO: 3 multithread the program.
# TODO: 2 Generally improve the coding.
# TODO: 3 Make it possible to set a ROI on the camera; so that the cameras only read the pixels in the ROI. This will
#  likely give us quite a bit of speed up on the program update time, and it is way simpler than multithreading.
# TODO: 3 Improve communication with the piezo motors.
# TODO: 2 Log the information on Grafana.
# TODO: 3 Move all print statements to the GUI.
# TODO: 2 Could we cash the current program settings to load automatically on next open?
# TODO: 2 Add GUI ability to set the averager state of the BOSON camera, and power on defaults.
# TODO: 3 It would be nice to allow the user to switch between IR and visibile system, which would require disconnecting
#  devices and reinitializing the cam_list.
# TODO: 1 I need to be able to run multiple instances of the program; so that I can run an IR and a vis instance in
#  parallel. 

if __name__ == "__main__":
    import sys
    import visa
    import time
    import numpy as np
    import pyqtgraph as pg
    from Packages.pointing_ui import Ui_MainWindow
    from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
    from Packages.camera import MightexCamera, MightexEngine, DeviceNotFoundError, BosonCamera
    from Packages.motors import MDT693A_Motor
    import tkinter as tk
    from tkinter import filedialog
    from serial.tools import list_ports
    from Packages.UpdateManager import UpdateManager
    from Packages.UpdateManager import InsufficientInformation

    STATE_MEASURE = 0
    STATE_CALIBRATE = 1
    STATE_LOCKED = 2
    STATE_ALIGN = 3
    state = STATE_MEASURE

    pg.setConfigOptions(imageAxisOrder='row-major')
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')

    UPDATE_TIME = 500  # ms

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    cam_model = QtGui.QStandardItemModel()
    motor_model = QtGui.QStandardItemModel()
    error_dialog = QtWidgets.QErrorMessage()

    #  Instantiate Update Manager
    UpdateManager = UpdateManager()

    # Find motors using VISA
    resourceManager = visa.ResourceManager()
    for dev in resourceManager.list_resources():
        motor_model.appendRow(QtGui.QStandardItem(str(dev)))
    motor_list = []
    """
    # Add fake cameras
    for i in range(3):
        c = FakeCamera()
        cam_list.append(c)
        cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
    """
    
    # Set models and default values for combo boxes
    ui.cb_cam1.setModel(cam_model)
    ui.cb_cam2.setModel(cam_model)
    ui.cb_motors_1.setModel(motor_model)
    ui.cb_motors_2.setModel(motor_model)
    if len(resourceManager.list_resources()) > 1:
        ui.cb_motors_1.setCurrentIndex(0)
        ui.cb_motors_2.setCurrentIndex(1)
    else:
        ui.cb_motors_1.setCurrentIndex(-1)
        ui.cb_motors_1.setCurrentIndex(-1)

    # Set the camera settings buttons invisible, until a Point Lock System is chosen. Then, make visible the correct
    # set of gui buttons.
    def toggle_mightex_cam_settings_ui_vis(Logic):
        """
        Set visibility state of Mightex camera settings UI to true or false. i.e. visibility of settings
        relavent to only Mightex cameras.
        input: True/False
        """
        ui.label.setVisible(Logic)
        ui.label_2.setVisible(Logic)
        ui.le_cam1_exp_time.setVisible(Logic)
        ui.le_cam1_gain.setVisible(Logic)
        ui.cb_cam1_decimate.setVisible(Logic)
        ui.label_5.setVisible(Logic)
        ui.label_6.setVisible(Logic)
        ui.le_cam2_exp_time.setVisible(Logic)
        ui.le_cam2_gain.setVisible(Logic)
        ui.cb_cam2_decimate.setVisible(Logic)

    def toggle_general_cam_settings_ui_vis(Logic):
        """
        Set visibility state of generally useful camera settings UI to true or false. i.e. visibility of settings
        relavent to both Mightex and Boson cameras.
        input: True/False
        """
        ui.btn_cam1_update.setVisible(Logic)
        ui.btn_cam2_update.setVisible(Logic)
        ui.label_7.setVisible(Logic)
        ui.label_8.setVisible(Logic)
        ui.le_cam1_threshold.setVisible(Logic)
        ui.le_cam2_threshold.setVisible(Logic)
        ui.label_4.setVisible(Logic)
        ui.label_3.setVisible(Logic)
        ui.label_7.setVisible(Logic)
        ui.cb_cam1.setVisible(Logic)
        ui.cb_cam2.setVisible(Logic)
        ui.label_14.setVisible(Logic)
        ui.label_15.setVisible(Logic)
        ui.Cam1_AvgFrames.setVisible(Logic)
        ui.Cam2_AvgFrames.setVisible(Logic)
        ui.label_9.setVisible(Logic)
        ui.label_10.setVisible(Logic)
        ui.le_cam1_max.setVisible(Logic)
        ui.le_cam2_max.setVisible(Logic)

    def toggle_BOSON_cam_settings_ui_vis(Logic):
        """
        Set visibility state of generally useful camera settings UI to true or false. i.e. visibility of settings
        relavent to both Mightex and Boson cameras.
        input: True/False
        """
        ui.label_16.setVisible(Logic)
        ui.label_17.setVisible(Logic)
        ui.label_18.setVisible(Logic)
        ui.label_19.setVisible(Logic)

    toggle_mightex_cam_settings_ui_vis(False)
    toggle_general_cam_settings_ui_vis(False)
    toggle_BOSON_cam_settings_ui_vis(False)

    # Load Most Recent Calibration Matrix:
    try:
        UpdateManager.calibration_matrix = np.loadtxt('Most_Recent_Calibration.txt', dtype=float)
    except OSError:
        raise FileNotFoundError("Hmm there seems to be no saved calibration, run calibration.")

    # Initialize Global variables for home position markers:
    ROICam1_Unlock = None
    ROICam2_Unlock = None
    ROICam1_Lock = None
    ROICam2_Lock = None

    # Load the Most Recent Home Position:
    def set_home_marker():
        global state
        global ROICam1_Unlock,ROICam2_Unlock,ROICam1_Lock,ROICam2_Lock
        set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y = UpdateManager.set_pos
        pen = pg.mkPen(color=(255, 0, 0), width=2)
        ROICam1_Unlock = pg.CircleROI(pos=(set_cam1_y-20, set_cam1_x-20), radius=20,
                                movable=False, rotatable=False, resizable=False, pen=pen)

        ROICam2_Unlock = pg.CircleROI(pos=(set_cam2_y-20, set_cam2_x-20), radius=20,
                                  movable=False, rotatable=False, resizable=False, pen=pen)
        ui.gv_camera1.addItem(ROICam1_Unlock)
        ui.gv_camera2.getView().addItem(ROICam2_Unlock)
        pen = pg.mkPen(color=(0, 255, 0), width=2)
        ROICam1_Lock = pg.CircleROI(pos=(set_cam1_y - 20, set_cam1_x - 20), radius=20,
                               movable=False, rotatable=False, resizable=False, pen=pen)

        ROICam2_Lock = pg.CircleROI(pos=(set_cam2_y - 20, set_cam2_x - 20), radius=20,
                               movable=False, rotatable=False, resizable=False, pen=pen)
        ui.gv_camera1.addItem(ROICam1_Lock)
        ui.gv_camera2.getView().addItem(ROICam2_Lock)
        if state == STATE_MEASURE:
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
        elif state == STATE_CALIBRATE:
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(False)
            ROICam2_Unlock.setVisible(False)
        elif state == STATE_LOCKED:
            ROICam1_Unlock.setVisible(False)
            ROICam2_Unlock.setVisible(False)
            ROICam1_Lock.setVisible(True)
            ROICam2_Lock.setVisible(True)
        elif state == STATE_ALIGN:
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
    try:
        HomePosition = np.loadtxt('Most_Recent_Home.txt', dtype=float)
        UpdateManager.set_pos = np.asarray(HomePosition)
        set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y = HomePosition
        set_home_marker()
    except OSError:
        set_cam1_x = 'dummy'
        set_cam1_y = 'dummy'
        set_cam2_x = 'dummy'
        set_cam2_y = 'dummy'
        print("Hmm there seems to be no saved Home Position, define one before locking.")

    # Initialize Global Variables for arrows for Alignment Mode:
    Cam1_LeftArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam1_RightArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam1_RightArrow.setRotation(180)
    Cam1_DownArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam1_DownArrow.setRotation(-90)
    Cam1_UpArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam1_UpArrow.setRotation(90)

    Cam2_LeftArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam2_RightArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam2_RightArrow.setRotation(180)
    Cam2_DownArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam2_DownArrow.setRotation(-90)
    Cam2_UpArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
    Cam2_UpArrow.setRotation(90)

    Cam1_LeftArrow.setScale(3)
    Cam1_RightArrow.setScale(3)
    Cam1_DownArrow.setScale(3)
    Cam1_UpArrow.setScale(3)
    Cam2_LeftArrow.setScale(3)
    Cam2_RightArrow.setScale(3)
    Cam2_DownArrow.setScale(3)
    Cam2_UpArrow.setScale(3)

    Cam1_LeftArrow.setPos(set_cam1_y + 40, set_cam1_x - 95)
    Cam1_RightArrow.setPos(set_cam1_y - 40, set_cam1_x + 92)
    Cam1_DownArrow.setPos(set_cam1_y - 95, set_cam1_x - 40)
    Cam1_UpArrow.setPos(set_cam1_y + 95, set_cam1_x + 40)

    Cam2_LeftArrow.setPos(set_cam2_y + 40, set_cam2_x - 95)
    Cam2_RightArrow.setPos(set_cam2_y - 40, set_cam2_x + 92)
    Cam2_DownArrow.setPos(set_cam2_y - 95, set_cam2_x - 40)
    Cam2_UpArrow.setPos(set_cam2_y + 95, set_cam2_x + 40)

    ui.gv_camera1.addItem(Cam1_LeftArrow)
    ui.gv_camera1.addItem(Cam1_RightArrow)
    ui.gv_camera1.addItem(Cam1_DownArrow)
    ui.gv_camera1.addItem(Cam1_UpArrow)
    ui.gv_camera2.getView().addItem(Cam2_LeftArrow)
    ui.gv_camera2.getView().addItem(Cam2_RightArrow)
    ui.gv_camera2.getView().addItem(Cam2_DownArrow)
    ui.gv_camera2.getView().addItem(Cam2_UpArrow)

    Cam1_LeftArrow.setVisible(False)
    Cam1_RightArrow.setVisible(False)
    Cam1_DownArrow.setVisible(False)
    Cam1_UpArrow.setVisible(False)
    Cam2_LeftArrow.setVisible(False)
    Cam2_RightArrow.setVisible(False)
    Cam2_DownArrow.setVisible(False)
    Cam2_UpArrow.setVisible(False)

    # Initialize global variables for tracking pointing
    cam1_x = np.zeros(1)
    cam1_y = np.zeros(1)
    cam2_x = np.zeros(1)
    cam2_y = np.zeros(1)
    cam1_x_time = np.zeros(1)
    cam1_y_time = np.zeros(1)
    cam2_x_time = np.zeros(1)
    cam2_y_time = np.zeros(1)
    cam1_x_plot = ui.gv_cam_xy.addPlot(row=0, col=0, labels={'left': 'Cam 1 X'}).plot()
    cam1_y_plot = ui.gv_cam_xy.addPlot(row=1, col=0, labels={'left': 'Cam 1 Y'}).plot()
    cam2_x_plot = ui.gv_cam_xy.addPlot(row=2, col=0, labels={'left': 'Cam 2 X'}).plot()
    cam2_y_plot = ui.gv_cam_xy.addPlot(row=3, col=0, labels={'left': 'Cam 2 Y'}).plot()
    # Initialize global variables for piezo motor voltages
    motor1_x = np.zeros(1)
    motor1_y = np.zeros(1)
    motor2_x = np.zeros(1)
    motor2_y = np.zeros(1)
    motor1_x_plot = ui.gv_piezo.addPlot(row=0, col=0).plot()
    motor1_y_plot = ui.gv_piezo.addPlot(row=1, col=0).plot()
    motor2_x_plot = ui.gv_piezo.addPlot(row=2, col=0).plot()
    motor2_y_plot = ui.gv_piezo.addPlot(row=3, col=0).plot()

    ## Set a custom color map
    colors = [
        (68, 1, 84),
        (72, 33, 115),
        (67, 62, 133),
        (56, 88, 140),
        (45, 112, 142),
        (37, 133, 142),
        (30, 155, 138),
        (42, 176, 127),
        (82, 197, 105),
        (134, 213, 73),
        (194, 223, 35),
        (253, 231, 37)
    ]
    cmap = pg.ColorMap(pos=np.linspace(0.0, 1.0, 12), color=colors)
    ui.gv_camera1.setColorMap(cmap)
    ui.gv_camera2.setColorMap(cmap)
    # Set the com guide lines
    cam1_x_line = pg.InfiniteLine(movable=False, angle=0)
    cam1_y_line = pg.InfiniteLine(movable=False, angle=90)
    cam2_x_line = pg.InfiniteLine(movable=False, angle=0)
    cam2_y_line = pg.InfiniteLine(movable=False, angle=90)
    ui.gv_camera1.addItem(cam1_x_line)
    ui.gv_camera1.addItem(cam1_y_line)
    ui.gv_camera2.getView().addItem(cam2_x_line)
    ui.gv_camera2.getView().addItem(cam2_y_line)
    cam_lines = [cam1_x_line, cam1_y_line, cam2_x_line, cam2_y_line]
    for l in cam_lines:
        l.setVisible(False)

    # Initialize global variables
    cam1_index = -1
    cam2_index = -1
    cam1_threshold = 0
    cam2_threshold = 0
    cam1_reset = True
    cam2_reset = True

    # Initialize global variables for Locking
    LockTimeStart = 0

    saturated_cam1 = False
    under_saturated_cam1 = False
    saturated_cam2 = False
    under_saturated_cam2 = False

    # Initialize global variables for calibration
    calib_index = 0
    reset_piezo = 0
    override = False
    calibration_voltages = 10 * np.arange(15)+5
    mot1_x_voltage, mot1_y_voltage, mot2_x_voltage, mot2_y_voltage = np.empty((4, len(calibration_voltages)))
    mot1_x_cam1_x, mot1_x_cam1_y, mot1_x_cam2_x, mot1_x_cam2_y = np.empty((4, len(calibration_voltages)))
    mot1_y_cam1_x, mot1_y_cam1_y, mot1_y_cam2_x, mot1_y_cam2_y = np.empty((4, len(calibration_voltages)))
    mot2_x_cam1_x, mot2_x_cam1_y, mot2_x_cam2_x, mot2_x_cam2_y = np.empty((4, len(calibration_voltages)))
    mot2_y_cam1_x, mot2_y_cam1_y, mot2_y_cam2_x, mot2_y_cam2_y = np.empty((4, len(calibration_voltages)))
    starting_v = 75.0*np.ones(4)

    def GUI_update_std(std):
        ui.le_cam1_dx_std.setText("{:.5f}".format(std[0]))
        ui.le_cam1_dy_std.setText("{:.5f}".format(std[1]))
        ui.le_cam2_dx_std.setText("{:.5f}".format(std[2]))
        ui.le_cam2_dy_std.setText("{:.5f}".format(std[3]))

    def resetHist(ref, min=0, max=255):
        """
        Given a reference to an image view, set the histogram's range and levels to min and max
        """
        assert isinstance(ref, pg.ImageView)
        ref.getHistogramWidget().setHistogramRange(min, max)
        ref.getHistogramWidget().setLevels(min, max)

    def addToPlot(data, plot, point, maxSize=100):
        """
        Scrolling plot with a maximum size
        """
        if (data.size < maxSize):
            data = np.append(data, point)
        else:
            data = np.roll(data, -1)
            data[-1] = point
            data = np.roll(data, -1)
            data[-1] = point
        plot.setData(data)
        return data

    def calc_com_x(img):
        Nx, Ny = img.shape
        x = np.arange(Nx)
        y = np.arange(Ny)
        X, _ = np.meshgrid(x, y, indexing='ij')
        w = img/np.sum(img)
        com_x = np.sum(X*w)
        return com_x

    def calc_com_y(img):
        Nx, Ny = img.shape
        x = np.arange(Nx)
        y = np.arange(Ny)
        _, Y = np.meshgrid(x, y, indexing='ij')
        w = img/np.sum(img)
        com_y = np.sum(Y*w)
        return com_y

    def calc_com(img):
        """
        Given an image, img, calculate the center of mass in pixel coordinates
        """
        Nx, Ny = img.shape
        x = np.arange(Nx)
        y = np.arange(Ny)
        X, Y = np.meshgrid(x, y, indexing='ij')
        try:
            w = img/np.sum(img)
        except RuntimeWarning:
            return 0, 0
        com_x = np.sum(X*w)
        com_y = np.sum(Y*w)
        return com_x, com_y

    def calc_com_opt(img):
        """Given an img, calculate the com in pixel coordinates. Algorithm slightly
        optimized by 1) calculating the maximum, 2) estimating the width by finding 
        the max/4 position, and 3) cropping the window to the max/4 position and
        calculating com
        """
        i, j = np.unravel_index(np.argmax(img), img.shape)
        r = np.abs(i - np.argmin(np.abs(img[i, j]/4 - img[:, j])))
        start_i = max(0, i - r)
        end_i = min(img.shape[0] - 1, i + r)
        start_j = max(0, j - r)
        end_j = min(img.shape[1] - 1, j + r)
        com_x, com_y = calc_com(img[start_i:end_i, start_j:end_j])
        return com_x + start_i, com_y + start_j

    def update():
        global LockTimeStart
        global motor_list
        global msg
        global start_time
        start_time = time.time()
        msg = ''
        if state == STATE_MEASURE:
            updateMeasure()
            msg += 'UNLOCKED'
        elif state == STATE_CALIBRATE:
            updateCalibrate()
            msg += 'Calibrating...'
        elif state == STATE_LOCKED:
            updateLocked()
            msg += 'LOCKED'
        elif state == STATE_ALIGN:
            updateAlign()
            msg += 'Alignment Mode'
        else:
            msg += 'ERROR'
        ui.statusbar.showMessage('{0}\tUpdate time: {1:.3f} (s)'.format(msg, time.time() - start_time))

    def take_img_calibration(cam_index, cam_view=0, threshold=0):
        global state
        global cam1_x, cam2_x, cam1_y, cam2_y
        global cam1_x_line, cam2_x_line, cam1_y_line, cam2_y_line
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        if int(ui.cb_SystemSelection.currentIndex()) == 2:
            service_BOSON(cam_index)
        if cam_view == 0:
            avg_frames = int(ui.Cam1_AvgFrames.text())
        else:
            avg_frames = int(ui.Cam2_AvgFrames.text())
        # Get an image that is an average over the number of AvgFrames
        if avg_frames < 2:
            try:
                cam_list[cam_index].update_frame()
                img = cam_list[cam_index].get_frame()
            except RuntimeError:
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
                img = np.array([1])
        else:
            for i in range(avg_frames):
                try:
                    cam_list[cam_index].update_frame()
                    temp_img = cam_list[cam_index].get_frame()
                except RuntimeError:
                    shut_down()
                    state = STATE_MEASURE
                    ROICam1_Lock.setVisible(False)
                    ROICam2_Lock.setVisible(False)
                    ROICam1_Unlock.setVisible(True)
                    ROICam2_Unlock.setVisible(True)
                    temp_img = np.array([1])
                    img = np.array([1])
                if i == 0:
                    img = temp_img
                else:
                    img += temp_img
            img = img/avg_frames
        if threshold > 0:
            img[img < threshold*avg_frames] = 0
        if cam_view == 0:
            ui.le_cam1_max.setText(str(np.max(img/avg_frames))) #Just updates the GUI to say what the max value of the camera is
        else:
            ui.le_cam2_max.setText(str(np.max(img/avg_frames))) #Just updates the GUI to say what the max value of the camera is
        com = calc_com(img)  # Grab COM

        # The below just plots the last 100 points of the COM for x and y
        if cam_view == 0:  # Decide whether cam 0 or 1, which camera?
            #x
            cam1_x_line.setPos(com[0])
            cam1_x_line.setVisible(True)
            if (cam1_x.size < int(ui.le_num_points.text())):
                cam1_x = np.append(cam1_x, com[0])
            else:
                cam1_x = np.roll(cam1_x, -1)
                cam1_x[-1] = com[0]
            cam1_x_plot.setData(cam1_x)
            #y
            cam1_y_line.setPos(com[1])
            cam1_y_line.setVisible(True)
            if (cam1_y.size < int(ui.le_num_points.text())):
                cam1_y = np.append(cam1_y, com[1])
            else:
                cam1_y = np.roll(cam1_y, -1)
                cam1_y[-1] = com[1]
            cam1_y_plot.setData(cam1_y)
        # Last 100 points of COM have been plotted, removing old points to maintain just 100.
        # Ditto the above for camera 2 below
        else:
            #x
            cam2_x_line.setPos(com[0])
            cam2_x_line.setVisible(True)
            if (cam2_x.size < int(ui.le_num_points.text())):
                cam2_x = np.append(cam2_x, com[0])
            else:
                cam2_x = np.roll(cam2_x, -1)
                cam2_x[-1] = com[0]
            cam2_x_plot.setData(cam2_x)
            # y
            cam2_y_line.setPos(com[1])
            cam2_y_line.setVisible(True)
            if (cam2_y.size < int(ui.le_num_points.text())):
                cam2_y = np.append(cam2_y, com[1])
            else:
                cam2_y = np.roll(cam2_y, -1)
                cam2_y[-1] = com[1]
            cam2_y_plot.setData(cam2_y)
        return img, com

    def take_img(cam_index, cam_view=0, threshold=0, resetView=False):
        """
        Given the camera at cam_index in the camera list
        Update cam_view with its image and COM
        Use threshold to zero out values below threshold
        Return if the image is saturated
        """
        global cam1_x_line, cam1_y_line, cam1_x, cam1_y
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global state
        global cam2_x_line, cam2_y_line, cam2_x, cam2_y
        global cam1_x, cam1_y, cam2_x, cam2_y
        global cam1_x_plot, cam1_y_plot, cam2_x_plot, cam2_y_plot
        assert cam_index >= 0
        assert cam_view == 0 or cam_view == 1

        if int(ui.cb_SystemSelection.currentIndex()) == 2:
            service_BOSON(cam_index)
        if cam_view == 0:
            avg_frames = int(ui.Cam1_AvgFrames.text())
        else:
            avg_frames = int(ui.Cam2_AvgFrames.text())
        # Get an image that is an average over the number of AvgFrames
        if avg_frames < 2:
            try:
                cam_list[cam_index].update_frame()
                img = cam_list[cam_index].get_frame()
            except RuntimeError:
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
                img = np.array([1])
        else:
            for i in range(avg_frames):
                try:
                    cam_list[cam_index].update_frame()
                    temp_img = cam_list[cam_index].get_frame()
                except RuntimeError:
                    shut_down()
                    state = STATE_MEASURE
                    ROICam1_Lock.setVisible(False)
                    ROICam2_Lock.setVisible(False)
                    ROICam1_Unlock.setVisible(True)
                    ROICam2_Unlock.setVisible(True)
                    temp_img = np.array([1])
                    img = np.array([1])
                if i==0:
                    img = temp_img
                else:
                    img += temp_img
            img = img/avg_frames
        if threshold > 0:
            img[img < threshold*avg_frames] = 0
        if cam_view == 0:
            ui.le_cam1_max.setText(str(np.max(img/avg_frames))) #Just updates the GUI to say what the max value of the camera is
            if cam1_index>=0 and cam2_index>=0:
                UpdateManager.cam_1_img = img
                UpdateManager.t1 = cam_list[cam_index].time
        else:
            ui.le_cam2_max.setText(str(np.max(img/avg_frames))) #Just updates the GUI to say what the max value of the camera is
            if cam1_index >= 0 and cam2_index >= 0:
                UpdateManager.cam_2_img = img
                UpdateManager.t2 = cam_list[cam_index].time
        com = calc_com(img)  # Grab COM
        under_saturated = np.all(img < 50)
        saturated = np.any(img > 250)
        if cam_view == 0:
            UpdateManager.cam_1_com = np.asarray(com)
            ui.le_cam1_max.setText(str(np.max(img)))
            cam_x_line = cam1_x_line
            cam_y_line = cam1_y_line
            cam_x_data = cam1_x
            cam_y_data = cam1_y
            cam_x_plot = cam1_x_plot
            cam_y_plot = cam1_y_plot
            gv_camera = ui.gv_camera1
        else:
            UpdateManager.cam_2_com = np.asarray(com)
            ui.le_cam2_max.setText(str(np.max(img)))
            cam_x_line = cam2_x_line
            cam_y_line = cam2_y_line
            cam_x_data = cam2_x
            cam_y_data = cam2_y
            cam_x_plot = cam2_x_plot
            cam_y_plot = cam2_y_plot
            gv_camera = ui.gv_camera2
        # Calculate COM
        # TODO: 3 optimize COM calculation by slicing window near the max value
        com_x = com[0]
        com_y = com[1]
        cam_x_line.setPos(com_x)
        cam_y_line.setPos(com_y)
        cam_x_line.setVisible(True)
        cam_y_line.setVisible(True)
        if (cam_x_data.size < int(ui.le_num_points.text())):
            cam_x_data = np.append(cam_x_data, com_x)
            cam_y_data = np.append(cam_y_data, com_y)
        else:
            cam_x_data = np.roll(cam_x_data, -1)
            cam_x_data[-1] = com_x
            cam_y_data = np.roll(cam_y_data, -1)
            cam_y_data[-1] = com_y
        if cam_view == 0:
            cam1_x = cam_x_data
            cam1_y = cam_y_data
        else:
            cam2_x = cam_x_data
            cam2_y = cam_y_data
        cam_x_plot.setData(cam_x_data)
        cam_y_plot.setData(cam_y_data)
        if resetView:
            gv_camera.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False)
        else:
            gv_camera.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
        return img, com_x, com_y, under_saturated, saturated

    def updateCalibrate():
        global state, calib_index, cam1_threshold, cam2_threshold
        global cam1_index, cam2_index
        global calibration_voltages, starting_v
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global override
        global mot1_x_voltage, mot1_y_voltage, mot2_x_voltage, mot2_y_voltage
        # mot represents the motor number, which is 1 or 2
        # cam represents the camera we're looking at, which is either near or far
        # x or y represents whether we are looking at the horizontal or vertical
        global mot1_x_cam1_x, mot1_x_cam1_y, mot1_x_cam2_x, mot1_x_cam2_y
        global mot1_y_cam1_x, mot1_y_cam1_y, mot1_y_cam2_x, mot1_y_cam2_y
        global mot2_x_cam1_x, mot2_x_cam1_y, mot2_x_cam2_x, mot2_x_cam2_y
        global mot2_y_cam1_x, mot2_y_cam1_y, mot2_y_cam2_x, mot2_y_cam2_y
        global motor_list
        global motor1_x, motor1_y, motor2_x, motor2_y
        global motor1_x_plot, motor1_y_plot, motor2_x_plot, motor2_y_plot
        global reset_piezo  #Switch variable.
        if (ui.cb_motors_1.currentIndex() < 0):
            error_dialog.showMessage('You need to select Motor 1.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        elif (ui.cb_motors_2.currentIndex() < 0):
            error_dialog.showMessage('You need to select Motor 2.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        elif ui.cb_motors_1.currentIndex() == ui.cb_motors_2.currentIndex():
            error_dialog.showMessage('You need to select two unique motors.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        if cam1_index < 0:
            error_dialog.showMessage('You need to select camera 1.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        elif cam2_index < 0:
            error_dialog.showMessage('You need to select camera 2.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        elif cam1_index == cam2_index:
            error_dialog.showMessage('You need to select two different cameras.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        else:
            num_steps_per_motor = len(calibration_voltages)
            if calib_index < num_steps_per_motor * 4:
                motor_index = calib_index // num_steps_per_motor
                voltage_step = calib_index % num_steps_per_motor
                set_voltage = calibration_voltages[voltage_step]
                if motor_index == 0:
                    # first motor, channel 1
                    voltage = motor_list[0].ch1_v
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        time.sleep(0.2) #Give time between setting the piezo and measuring the COM
                        override = False
                        reset_piezo = 0
                        # update motor voltages
                        motor1_x = addToPlot(motor1_x, motor1_x_plot, voltage)
                        # get a frame from cam1 for x motor
                        img, com = take_img_calibration(cam1_index, cam_view = 0, threshold = cam1_threshold)
                        # put in list, update image
                        mot1_x_cam1_x[voltage_step] = com[0]
                        mot1_x_cam1_y[voltage_step] = com[1]
                        ui.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2 for x motor
                        img, com = take_img_calibration(cam2_index, cam_view=1, threshold = cam2_threshold)
                        # put in list
                        mot1_x_cam2_x[voltage_step] = com[0]
                        mot1_x_cam2_y[voltage_step] = com[1]
                        ui.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot1_x_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        #If the voltage is not where it should be, update it.
                        motor_list[0].ch1_v = set_voltage
                        if reset_piezo > 0: # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
                elif motor_index == 1:
                    motor_list[0].ch1_v = starting_v[0]
                    # first motor, channel 2
                    voltage = motor_list[0].ch2_v
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        time.sleep(0.2) #Give time between setting the piezo and measuring the COM
                        override = False
                        reset_piezo = 0
                        # update motor voltages
                        motor1_y = addToPlot(motor1_y, motor1_y_plot, voltage)
                        # get a frame from cam1
                        img, com = take_img_calibration(cam1_index, cam_view = 0, threshold = cam1_threshold, avg_frames = 2, avg_com = 2)
                        # put in list
                        mot1_y_cam1_x[voltage_step] = com[0]
                        mot1_y_cam1_y[voltage_step] = com[1]
                        ui.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2
                        img, com = take_img_calibration(cam2_index, cam_view = 1, threshold = cam2_threshold, avg_frames = 2, avg_com = 2)
                        # put in list
                        mot1_y_cam2_x[voltage_step] = com[0]
                        mot1_y_cam2_y[voltage_step] = com[1]
                        ui.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot1_y_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        # If the voltage is not where it should be, update it.
                        motor_list[0].ch2_v = set_voltage
                        if reset_piezo > 0:  # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
                elif motor_index == 2:
                    motor_list[0].ch2_v = starting_v[1]
                    # second motor, channel 1
                    voltage = motor_list[1].ch1_v
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        time.sleep(0.2) #Give time between setting the piezo and measuring the COM
                        reset_piezo = 0
                        override = False
                        # update motor voltages
                        motor2_x = addToPlot(motor2_x, motor2_x_plot, voltage)
                        # get a frame from cam1 for x motor
                        img, com = take_img_calibration(cam1_index, cam_view = 0, threshold = cam1_threshold, avg_frames = 2, avg_com = 2)
                        # put in list, update image
                        mot2_x_cam1_x[voltage_step] = com[0]
                        mot2_x_cam1_y[voltage_step] = com[1]
                        ui.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2 for x motor
                        img, com = take_img_calibration(cam2_index, cam_view = 1, threshold = cam2_threshold, avg_frames = 2, avg_com = 2)
                        # put in list
                        mot2_x_cam2_x[voltage_step] = com[0]
                        mot2_x_cam2_y[voltage_step] = com[1]
                        ui.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot2_x_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        # If the voltage is not where it should be, update it.
                        motor_list[1].ch1_v = set_voltage
                        if reset_piezo > 0:  # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
                elif motor_index == 3:
                    motor_list[1].ch1_v = starting_v[2]
                    # second motor, channel 2
                    voltage = motor_list[1].ch2_v
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                        reset_piezo = 0
                        override = False
                        # update motor voltages
                        motor2_y = addToPlot(motor2_y, motor2_y_plot, voltage)
                        # get a frame from cam1
                        img, com = take_img_calibration(cam1_index, cam_view = 0, threshold = cam1_threshold, avg_frames = 2, avg_com = 2)
                        # put in list
                        mot2_y_cam1_x[voltage_step] = com[0]
                        mot2_y_cam1_y[voltage_step] = com[1]
                        ui.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2
                        img, com = take_img_calibration(cam2_index, cam_view = 1, threshold = cam2_threshold, avg_frames = 2, avg_com = 2)
                        # put in list
                        mot2_y_cam2_x[voltage_step] = com[0]
                        mot2_y_cam2_y[voltage_step] = com[1]
                        ui.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot2_y_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        # If the voltage is not where it should be, update it.
                        motor_list[1].ch2_v = set_voltage
                        if reset_piezo > 0:  # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
            else:
                # reset only last motor (the others were reset before moving next motor)
                motor_list[1].ch2_v = starting_v[3]
                # calculate slopes
                p_mot1_x_cam1_x = np.polyfit(mot1_x_voltage, mot1_x_cam1_x, deg=1)
                p_mot1_x_cam2_x = np.polyfit(mot1_x_voltage, mot1_x_cam2_x, deg=1)
                p_mot1_x_cam1_y = np.polyfit(mot1_x_voltage, mot1_x_cam1_y, deg=1)
                p_mot1_x_cam2_y = np.polyfit(mot1_x_voltage, mot1_x_cam2_y, deg=1)
                p_mot1_y_cam1_x = np.polyfit(mot1_y_voltage, mot1_y_cam1_x, deg=1)
                p_mot1_y_cam2_x = np.polyfit(mot1_y_voltage, mot1_y_cam2_x, deg=1)
                p_mot1_y_cam1_y = np.polyfit(mot1_y_voltage, mot1_y_cam1_y, deg=1)
                p_mot1_y_cam2_y = np.polyfit(mot1_y_voltage, mot1_y_cam2_y, deg=1)
                p_mot2_x_cam1_x = np.polyfit(mot2_x_voltage, mot2_x_cam1_x, deg=1)
                p_mot2_x_cam2_x = np.polyfit(mot2_x_voltage, mot2_x_cam2_x, deg=1)
                p_mot2_x_cam1_y = np.polyfit(mot2_x_voltage, mot2_x_cam1_y, deg=1)
                p_mot2_x_cam2_y = np.polyfit(mot2_x_voltage, mot2_x_cam2_y, deg=1)
                p_mot2_y_cam1_x = np.polyfit(mot2_y_voltage, mot2_y_cam1_x, deg=1)
                p_mot2_y_cam2_x = np.polyfit(mot2_y_voltage, mot2_y_cam2_x, deg=1)
                p_mot2_y_cam1_y = np.polyfit(mot2_y_voltage, mot2_y_cam1_y, deg=1)
                p_mot2_y_cam2_y = np.polyfit(mot2_y_voltage, mot2_y_cam2_y, deg=1)

                '''
                construct calibration matrix:
                Understand that this matrix is dV_i/dx_j where ij indexes like a normal matrix, i.e. row column. 
                Therefore, dV = calib_mat * dx where dx is a column vector, [d_cam1_x, d_cam1_y, d_cam2_x, d_cam2_y].Transpose,
                where, for example, d_cam1_x is the difference in the desired position of cam1_x and the calculated COM x_coordinate of cam 1.
                and dV is a column vector, [d_mot1_x, d_mot1_y, d_mot2_x, d_mot2_y,].Transpose, i.e. the change in voltage
                 on each motor to bring the COM coordinates to desired position.
                Therefore, this matrix can be used to update the motor voltages as New_V = Old_V + calib_mat*dx 
                '''
                calib_mat = np.array([[p_mot1_x_cam1_x[0], p_mot1_y_cam1_x[0], p_mot2_x_cam1_x[0], p_mot2_y_cam1_x[0]],
                                        [p_mot1_x_cam1_y[0], p_mot1_y_cam1_y[0], p_mot2_x_cam1_y[0], p_mot2_y_cam1_y[0]],
                                        [p_mot1_x_cam2_x[0], p_mot1_y_cam2_x[0], p_mot2_x_cam2_x[0], p_mot2_y_cam2_x[0]],
                                        [p_mot1_x_cam2_y[0], p_mot1_y_cam2_y[0], p_mot2_x_cam2_y[0], p_mot2_y_cam2_y[0]]])
                UpdateManager.calibration_matrix = np.linalg.inv(calib_mat)
                try:
                    old_calib_mat = np.loadtxt('Most_Recent_Calibration.txt', dtype=float)
                    Change = np.sqrt(np.sum(np.square(calib_mat)-np.square(old_calib_mat)))
                    RelativeChange = Change/np.sqrt(np.sum(np.square(old_calib_mat)))
                    print(RelativeChange)
                except OSError:
                    pass
                print('Calibration done!')
                print(calib_mat)
                np.savetxt('Most_Recent_Calibration.txt', calib_mat, fmt='%f')
                filename = "CalibrationMatrixStored/" + str(np.datetime64('today', 'D')) + "_Calib_mat"
                np.savetxt(filename, calib_mat, fmt='%f')
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)

    def updateMeasure():
        """
        GUI update function
        """
        global saturated_cam1, under_saturated_cam1, saturated_cam2, under_saturated_cam2
        global cam1_reset, cam2_reset
        global cam1_x, cam1_y, cam2_x, cam2_y
        global cam1_x_time, cam1_y_time, cam2_x_time, cam2_y_time
        ############################
        # Camera 1 update function #
        ############################
        if cam1_index >= 0:
            if cam1_reset:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0, threshold=cam1_threshold, resetView=True)
                cam1_reset = False
            else:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0, threshold=cam1_threshold, resetView=False)
        else:
            cam1_x_line.setVisible(False)
            cam1_y_line.setVisible(False)
        ############################
        # Camera 2 update function #
        ############################
        if cam2_index >= 0:
            if cam2_reset:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1, threshold=cam2_threshold, resetView=True)
                cam2_reset = False
            else:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1, threshold=cam2_threshold, resetView=False)
        else:
            cam2_x_line.setVisible(False)
            cam2_y_line.setVisible(False)
        # Update dx with update manager if 2 cameras are connected:
        if cam1_index >=0 and cam2_index>=0:
            UpdateManager.calc_dx()
            GUI_update_std(UpdateManager.standard_deviation)

    def updateLocked():
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global motor1_x, motor1_y, motor2_x, motor2_y
        global motor1_x_plot, motor1_y_plot, motor2_x_plot, motor2_y_plot
        global LockTimeStart, TimeLastUnlock
        global num_out_of_voltage_range  # Track the number of times the voltage update goes out of range during lock
        global state
        global first_unlock
        global saturated_cam1, under_saturated_cam1, saturated_cam2, under_saturated_cam2
        global cam1_reset, cam2_reset
        global cam1_x_time, cam1_y_time, cam2_x_time, cam2_y_time
        if cam1_index >= 0 and cam2_index >= 0 and cam1_index != cam2_index:
            if cam1_reset:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0,
                                                                                   threshold=cam1_threshold,
                                                                                   resetView=True)
                cam1_reset = False
            else:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0,
                                                                                   threshold=cam1_threshold,
                                                                                   resetView=False)
            if cam2_reset:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1,
                                                                                   threshold=cam2_threshold,
                                                                                   resetView=True)
                cam2_reset = False
            else:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1,
                                                                                   threshold=cam2_threshold,
                                                                                   resetView=False)
            # Tell the update manager the current piezo voltages
            UpdateManager.V0 = np.array([motor_list[0].ch1_v, motor_list[0].ch2_v, motor_list[1].ch1_v,
                                       motor_list[1].ch2_v])
            try:
                # Try to ping the update manager for the correct update voltage.
                update_voltage = UpdateManager.get_update()
                # update voltages
                if np.any(update_voltage > 150) or np.any(update_voltage < 0):
                    # This section of code basically resets the voltages and skips the update this time, but it allows
                    # the code to try to relock after resetting the voltages. However, this code tracks the number of times
                    # that the voltages went out of bounds in less than 1 minute since the last reset (i.e. if it resets and
                    # then locks for more than 1 minute, the counter is reset to 0). If the piezos go out of bounds more than
                    # 10 times in a row in less than a minute each time, then the code unlocks and returns to measuring state.
                    print("Warning: The update voltages went out of range, resetting piezos and attempting to relock")
                    ROICam1_Lock.setVisible(False)
                    ROICam2_Lock.setVisible(False)
                    ROICam1_Unlock.setVisible(True)
                    ROICam2_Unlock.setVisible(True)
                    # Reset the voltages to 75.0 the OG settings, and skip this update. Try again.
                    motor_list[0].ch1_v = 75.0
                    motor_list[0].ch2_v = 75.0
                    motor_list[1].ch1_v = 75.0
                    motor_list[1].ch2_v = 75.0
                    ##############################
                    # Update Piezo voltage Plots #
                    ##############################
                    if (motor1_x.size < int(ui.le_num_points.text())):
                        motor1_x = np.append(motor1_x, 75.0)
                        motor1_y = np.append(motor1_y, 75.0)
                        motor2_x = np.append(motor2_x, 75.0)
                        motor2_y = np.append(motor2_y, 75.0)
                    else:
                        motor1_x = np.roll(motor1_x, -1)
                        motor1_x[-1] = 75.0
                        motor1_y = np.roll(motor1_y, -1)
                        motor1_y[-1] = 75.0
                        motor2_x = np.roll(motor2_x, -1)
                        motor2_x[-1] = 75.0
                        motor2_y = np.roll(motor2_y, -1)
                        motor2_y[-1] = 75.0
                    motor1_x_plot.setData(motor1_x)
                    motor1_y_plot.setData(motor1_y)
                    motor2_x_plot.setData(motor2_x)
                    motor2_y_plot.setData(motor2_y)

                    # Track time since last unlock and the number of times unlocked in less than 1 minute since previous
                    # lock.
                    if time.monotonic() - TimeLastUnlock < 60.0 or first_unlock:
                        first_unlock = False
                        num_out_of_voltage_range += 1
                        TimeLastUnlock = time.monotonic()
                    else:
                        TimeLastUnlock = time.monotonic()
                        num_out_of_voltage_range = 1
                    if num_out_of_voltage_range > 10:
                        num_out_of_voltage_range = 0
                        TimeLastUnlock = 0
                        shut_down()
                        state = STATE_MEASURE
                        ROICam1_Lock.setVisible(False)
                        ROICam2_Lock.setVisible(False)
                        ROICam1_Unlock.setVisible(True)
                        ROICam2_Unlock.setVisible(True)
                        successful_lock_time = time.monotonic() - LockTimeStart
                        print("Successfully locked pointing for", successful_lock_time)
                        raise NotImplementedError(
                            'The piezo voltages have gone outside of the allowed range! Stop Lock')
                else:
                    ROICam1_Unlock.setVisible(False)
                    ROICam2_Unlock.setVisible(False)
                    ROICam1_Lock.setVisible(True)
                    ROICam2_Lock.setVisible(True)
                    motor_list[0].ch1_v = update_voltage[0]
                    motor_list[0].ch2_v = update_voltage[1]
                    motor_list[1].ch1_v = update_voltage[2]
                    motor_list[1].ch2_v = update_voltage[3]

                    ##############################
                    # Update Piezo voltage Plots #
                    ##############################
                    if (motor1_x.size < int(ui.le_num_points.text())):
                        motor1_x = np.append(motor1_x, update_voltage[0])
                        motor1_y = np.append(motor1_y, update_voltage[1])
                        motor2_x = np.append(motor2_x, update_voltage[2])
                        motor2_y = np.append(motor2_y, update_voltage[3])
                    else:
                        motor1_x = np.roll(motor1_x, -1)
                        motor1_x[-1] = update_voltage[0]
                        motor1_y = np.roll(motor1_y, -1)
                        motor1_y[-1] = update_voltage[1]
                        motor2_x = np.roll(motor2_x, -1)
                        motor2_x[-1] = update_voltage[2]
                        motor2_y = np.roll(motor2_y, -1)
                        motor2_y[-1] = update_voltage[3]
                    motor1_x_plot.setData(motor1_x)
                    motor1_y_plot.setData(motor1_y)
                    motor2_x_plot.setData(motor2_x)
                    motor2_y_plot.setData(motor2_y)
                GUI_update_std(UpdateManager.standard_deviation)
            except InsufficientInformation:
                # catch exception and return to measure state.
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
                return

        else:
            if cam1_index < 0:
                cam1_x_line.setVisible(False)
                cam1_y_line.setVisible(False)
            if cam2_index < 0:
                cam2_x_line.setVisible(False)
                cam2_y_line.setVisible(False)

    def updateAlign():
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global Cam1_LeftArrow, Cam1_RightArrow, Cam1_DownArrow, Cam1_UpArrow
        global Cam2_LeftArrow, Cam2_RightArrow, Cam2_DownArrow, Cam2_UpArrow
        global motor1_x, motor1_y, motor2_x, motor2_y
        global motor1_x_plot, motor1_y_plot, motor2_x_plot, motor2_y_plot
        global LockTimeStart, TimeLastUnlock
        global state
        global saturated_cam1, under_saturated_cam1, saturated_cam2, under_saturated_cam2
        global cam1_reset, cam2_reset
        global cam1_x_time, cam1_y_time, cam2_x_time, cam2_y_time
        if cam1_index >= 0 and cam2_index >= 0 and cam1_index != cam2_index:
            if cam1_reset:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0,
                                                                                   threshold=cam1_threshold,
                                                                                   resetView=True)
                cam1_reset = False
            else:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0,
                                                                                   threshold=cam1_threshold,
                                                                                   resetView=False)
            if cam2_reset:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1,
                                                                                   threshold=cam2_threshold,
                                                                                   resetView=True)
                cam2_reset = False
            else:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1,
                                                                                   threshold=cam2_threshold,
                                                                                   resetView=False)
            try:
                update_voltage = UpdateManager.get_update()
            except InsufficientInformation:
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
                return

            # If the beams are within 10V displacements in any direction from 75.0 V, their center voltage, then make
            # the home circle green to let the user know. This is the condition when the user should stop trying to
            # align.
            #ToDO: 2 Make this threshold (the 10.0 below) setable in the GUI.
            if np.any(np.abs(update_voltage - 75.0)) > 10.0:
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
            else:
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)

            # The goal here is to provide the user feedback about which direction to steer the beam.
            # x and y are switched relative to your expectation of L/R up/down,
            # because the cameras happen to be rotated
            # ToDo: 2 Make the orientation of the Cameras setable in the GUI, without breaking the whole code.
            #  i.e. let me rotate and reflect the cameras as needed; so I can set L/R U/D to be consistent with true
            #  orientations.
            displacementThreshold = 1
            if UpdateManager.dx[-1, 0] > displacementThreshold:
                Cam1_DownArrow.setVisible(False)
                Cam1_UpArrow.setVisible(True)
            elif UpdateManager.dx[-1, 0] < -displacementThreshold:
                Cam1_UpArrow.setVisible(False)
                Cam1_DownArrow.setVisible(True)
            else:
                Cam1_UpArrow.setVisible(False)
                Cam1_DownArrow.setVisible(False)

            if UpdateManager.dx[-1, 1] > displacementThreshold:
                Cam1_RightArrow.setVisible(False)
                Cam1_LeftArrow.setVisible(True)
            elif UpdateManager.dx[-1, 1] < -displacementThreshold:
                Cam1_LeftArrow.setVisible(False)
                Cam1_RightArrow.setVisible(True)
            else:
                Cam1_LeftArrow.setVisible(False)
                Cam1_RightArrow.setVisible(False)

            if UpdateManager.dx[-1, 2] > displacementThreshold:
                Cam2_DownArrow.setVisible(False)
                Cam2_UpArrow.setVisible(True)
            elif UpdateManager.dx[-1, 2] < -displacementThreshold:
                Cam2_UpArrow.setVisible(False)
                Cam2_DownArrow.setVisible(True)
            else:
                Cam2_UpArrow.setVisible(False)
                Cam2_DownArrow.setVisible(False)

            if UpdateManager.dx[-1, 3] > displacementThreshold:
                Cam2_RightArrow.setVisible(False)
                Cam2_LeftArrow.setVisible(True)
            elif UpdateManager.dx[-1, 3] < -displacementThreshold:
                Cam2_LeftArrow.setVisible(False)
                Cam2_RightArrow.setVisible(True)
            else:
                Cam2_LeftArrow.setVisible(False)
                Cam2_RightArrow.setVisible(False)
        else:
            if cam1_index < 0:
                cam1_x_line.setVisible(False)
                cam1_y_line.setVisible(False)
            if cam2_index < 0:
                cam2_x_line.setVisible(False)
                cam2_y_line.setVisible(False)

    '''
    Archived...
    def updateLocked():
        global cailb_x, calib_y 
        global set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y
        global saturated_cam1, under_saturated_cam1, saturated_cam2, under_saturated_cam2
        global cam1_reset, cam2_reset
        global cam1_x_time, cam1_y_time, cam2_x_time, cam2_y_time
        if cam1_index >= 0 and cam2_index >= 0 and cam1_index != cam2_index:
            if cam1_reset:
                _, com1_x, com1_y, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0, threshold=cam1_threshold, resetView=True)
                cam1_reset = False
            else:
                _, com1_x, com1_y, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0, threshold=cam1_threshold, resetView=False)
            if cam2_reset:
                _, com2_x, com2_y, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1, threshold=cam2_threshold, resetView=True)
                cam2_reset = False
            else:
                _, com2_x, com2_y, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1, threshold=cam2_threshold, resetView=False)
            cam1_dx = set_cam1_x - com1_x
            cam1_dy = set_cam1_y - com1_y
            cam2_dx = set_cam2_x - com2_x
            cam2_dy = set_cam2_x - com2_y
            #These are changes in voltage, not the actual voltage to be applied?
            v_x = np.dot(calib_x, np.array([cam1_dx, cam2_dx]))
            v_y = np.dot(calib_y, np.array([cam1_dy, cam2_dy]))

            #update voltages
            motor_list[0].ch1_v = v_x[0]+motor_list[0]._ch1_v
            motor_list[1].ch1_v = v_x[1]+motor_list[1]._ch1_v
            motor_list[0].ch2_v = v_y[0]+motor_list[0]._ch2_v
            motor_list[1].ch2_v = v_y[0]+motor_list[1]._ch2_v
        else:
            if cam1_index < 0:
                cam1_x_line.setVisible(False)
                cam1_y_line.setVisible(False)
            if cam2_index < 0:
                cam2_x_line.setVisible(False)
                cam2_y_line.setVisible(False)
    '''

    def update_cam1_settings():
        global cam1_index, cam1_threshold, cam1_reset, cam_list
        if int(ui.cb_SystemSelection.currentIndex()) == 1:
            cam1_index = int(ui.cb_cam1.currentIndex())
            cam1_exp_time = float(ui.le_cam1_exp_time.text())
            cam1_gain = float(ui.le_cam1_gain.text())
            cam1_threshold = float(ui.le_cam1_threshold.text())
            cam1_decimate = ui.cb_cam1_decimate.isChecked()
            cam_list[cam1_index].set_gain(cam1_gain)
            cam_list[cam1_index].set_exposure_time(cam1_exp_time)
            cam_list[cam1_index].update_frame()
            cam_list[cam1_index].set_decimation(cam1_decimate)
            ui.le_cam1_exp_time.setText('%.2f' % (cam_list[cam1_index].exposure_time))
            ui.le_cam1_gain.setText('%.2f' % (cam_list[cam1_index].gain/8))
            cam1_reset = True
            resetHist(ui.gv_camera1)
        elif int(ui.cb_SystemSelection.currentIndex()) == 2:
            cam1_index = int(ui.cb_cam1.currentIndex())
            cam1_threshold = float(ui.le_cam1_threshold.text())
            cam_list[cam1_index].update_frame()
            cam1_reset = True
            resetHist(ui.gv_camera1, max=65536)
        else:
            print("Choose a Point Lock system first!")
    
    def update_cam2_settings():
        global cam2_index, cam2_threshold, cam2_reset, cam_list
        if int(ui.cb_SystemSelection.currentIndex()) == 1:
            cam2_index = int(ui.cb_cam2.currentIndex())
            cam2_exp_time = float(ui.le_cam2_exp_time.text())
            cam2_gain = float(ui.le_cam2_gain.text())
            cam2_threshold = float(ui.le_cam2_threshold.text())
            cam2_decimate = ui.cb_cam2_decimate.isChecked()
            cam_list[cam2_index].set_gain(cam2_gain)
            cam_list[cam2_index].set_exposure_time(cam2_exp_time)
            cam_list[cam1_index].update_frame()
            cam_list[cam2_index].set_decimation(cam2_decimate)
            ui.le_cam2_exp_time.setText('%.2f' % (cam_list[cam2_index].exposure_time))
            ui.le_cam2_gain.setText('%.2f' % (cam_list[cam2_index].gain/8))
            cam2_reset = True
            resetHist(ui.gv_camera2)
        elif int(ui.cb_SystemSelection.currentIndex()) == 2:
            cam2_index = int(ui.cb_cam2.currentIndex())
            cam2_threshold = float(ui.le_cam2_threshold.text())
            cam_list[cam1_index].update_frame()
            cam2_reset = True
            resetHist(ui.gv_camera2,max=65536)
        else:
            print("Choose a Point Lock system first!")
    
    def update_motors():
        global motor1_index, motor2_index
        global motor_list, resourceManager
        motor1_index = ui.cb_motors_1.currentIndex()
        motor2_index = ui.cb_motors_2.currentIndex()
        if motor1_index != motor2_index:
            motor_list = []
            #Garrison Updated to add "0" inside ui.cb_motors_1.currentData(0)
            motor_list.append(MDT693A_Motor(resourceManager, com_port=ui.cb_motors_1.currentData(0), ch1='X', ch2='Y'))
            motor_list.append(MDT693A_Motor(resourceManager, com_port=ui.cb_motors_2.currentData(0), ch1='X', ch2='Y'))
            #motor_list.append(MDT693A_Motor(resourceManager, com_port=ui.cb_motors_1.currentData(), ch1='X', ch2='Y'))
            #motor_list.append(MDT693A_Motor(resourceManager, com_port=ui.cb_motors_2.currentData(), ch1='X', ch2='Y'))
        #motor_list.append(FakeMotor('X', 'Y'))
        #motor_list.append(FakeMotor('X', 'Y'))
        #print('Connected two fake motors!')
    
    def begin_calibration():
        global state, starting_v, motor_list, calib_index
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        calib_index = 0
        if len(motor_list) == 0:
            update_motors()
        ROICam1_Unlock.setVisible(False)
        ROICam2_Unlock.setVisible(False)
        ROICam1_Lock.setVisible(False)
        ROICam2_Lock.setVisible(False)
        starting_v[0] = motor_list[0].ch1_v
        starting_v[1] = motor_list[0].ch2_v
        starting_v[2] = motor_list[1].ch1_v
        starting_v[3] = motor_list[1].ch1_v
        shut_down()
        state = STATE_CALIBRATE
    
    def clear_pointing_plots():
        global cam1_x, cam1_y, cam2_x, cam2_y
        cam1_x = np.array([])
        cam1_y = np.array([])
        cam2_x = np.array([])
        cam2_y = np.array([])
    
    def lock_pointing():
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global state
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global Cam1_LeftArrow, Cam1_RightArrow, Cam1_DownArrow, Cam1_UpArrow
        global Cam2_LeftArrow, Cam2_RightArrow, Cam2_DownArrow, Cam2_UpArrow
        global LockTimeStart, TimeLastUnlock, num_out_of_voltage_range
        global first_unlock
        Cam1_LeftArrow.setVisible(False)
        Cam1_RightArrow.setVisible(False)
        Cam1_DownArrow.setVisible(False)
        Cam1_UpArrow.setVisible(False)
        Cam2_LeftArrow.setVisible(False)
        Cam2_RightArrow.setVisible(False)
        Cam2_DownArrow.setVisible(False)
        Cam2_UpArrow.setVisible(False)
        if (ui.btn_lock.isChecked()):
            if cam1_index >= 0 and cam2_index >= 0:
                shut_down()
                state = STATE_LOCKED
                TimeLastUnlock = 0
                num_out_of_voltage_range = 0
                first_unlock = True  # First time since initial lock that piezos went out of bounds?
                LockTimeStart = time.monotonic()
                ROICam1_Unlock.setVisible(False)
                ROICam2_Unlock.setVisible(False)
                ROICam1_Lock.setVisible(True)
                ROICam2_Lock.setVisible(True)
                if len(motor_list) == 0:
                    update_motors()
            else:
                ui.btn_lock.toggle()
        else:
            if state != STATE_MEASURE:
                shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)

    def define_home():
        global cam1_x, cam1_y, cam2_x, cam2_y
        if cam1_index >= 0 and cam2_index >= 0:
            set_cam1_x = cam1_x[-1]
            set_cam1_y = cam1_y[-1]
            set_cam2_x = cam2_x[-1]
            set_cam2_y = cam2_y[-1]
            print('Set cam 1 x', set_cam1_x)
            print('Set cam 1 y', set_cam1_y)
            print('Set cam 2 x', set_cam2_x)
            print('Set cam 2 y', set_cam2_y)
            HomePosition = np.array([set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y])
            UpdateManager.set_pos = HomePosition
            if int(ui.cb_SystemSelection.currentIndex()) == 1:
                np.savetxt('Most_Recent_Home.txt', HomePosition, fmt='%f')
                filename = "HomePositionStored/" + str(np.datetime64('today', 'D')) + "_Home"
                np.savetxt(filename, HomePosition, fmt='%f')
            elif int(ui.cb_SystemSelection.currentIndex()) == 2:
                np.savetxt('Most_Recent_Home_IR.txt', HomePosition, fmt='%f')
                filename = "HomePositionStored/" + str(np.datetime64('today', 'D')) + "_Home_IR"
                np.savetxt(filename, HomePosition, fmt='%f')
            try:
                ROICam1_Unlock.setVisible(False)
                ROICam2_Unlock.setVisible(False)
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
            except:
                pass
            set_home_marker()

    def load_home():
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename()
        if int(ui.cb_SystemSelection.currentIndex()) == 1:
            HomePosition = np.loadtxt(file_path, dtype=float)
            np.savetxt('Most_Recent_Home.txt', HomePosition, fmt='%f')
        elif int(ui.cb_SystemSelection.currentIndex()) == 2:
            HomePosition = np.loadtxt(file_path, dtype=float)
            np.savetxt('Most_Recent_Home_IR.txt', HomePosition, fmt='%f')
        UpdateManager.set_pos = np.asarray(HomePosition)
        print("Set Positions:", HomePosition)
        try:
            ROICam1_Unlock.setVisible(False)
            ROICam2_Unlock.setVisible(False)
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
        except:
            pass
        set_home_marker()

    def begin_align():
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global Cam1_LeftArrow, Cam1_RightArrow, Cam1_DownArrow, Cam1_UpArrow
        global Cam2_LeftArrow, Cam2_RightArrow, Cam2_DownArrow, Cam2_UpArrow
        global state, start_time
        global msg
        shut_down()
        if state == STATE_ALIGN:
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            Cam1_LeftArrow.setVisible(False)
            Cam1_RightArrow.setVisible(False)
            Cam1_DownArrow.setVisible(False)
            Cam1_UpArrow.setVisible(False)
            Cam2_LeftArrow.setVisible(False)
            Cam2_RightArrow.setVisible(False)
            Cam2_DownArrow.setVisible(False)
            Cam2_UpArrow.setVisible(False)
        else:
            state = STATE_ALIGN
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            if len(motor_list) == 0:
                update_motors()
            try:
                motor_list[0].ch1_v = 75.0
                motor_list[0].ch2_v = 75.0
                motor_list[1].ch1_v = 75.0
                motor_list[1].ch2_v = 75.0
                UpdateManager.V0 = np.array([motor_list[0].ch1_v, motor_list[0].ch2_v, motor_list[1].ch1_v,
                                             motor_list[1].ch2_v])
            except:
                UpdateManager.V0 = np.array([motor_list[0].ch1_v, motor_list[0].ch2_v, motor_list[1].ch1_v,
                                             motor_list[1].ch2_v])
                msg += "WARNING! Alignment should be done w/ piezos at 75 V, but motors are not connected and " \
                       "voltages are unkown!"
                ui.statusbar.showMessage('{0}\tUpdate time: {1:.3f} (s)'.format(msg, time.time() - start_time))
                time.sleep(5)
                raise AssertionError("Make Piezo voltages 75.0 before aligning.")

    def find_cameras():
        global cam_list
        if int(ui.cb_SystemSelection.currentIndex()) == 1:
            # Find the Mightex cameras
            mightex_engine = MightexEngine()
            cam_list = []
            if len(mightex_engine.serial_no) == 0:
                raise DeviceNotFoundError('Could not find any Mightex cameras!')
            else:
                for i, serial_no in enumerate(mightex_engine.serial_no):
                    c = MightexCamera(mightex_engine, serial_no)
                    cam_list.append(c)
                    cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
            # Make appropriate Camera Settings available in GUI:
            toggle_BOSON_cam_settings_ui_vis(False)
            toggle_mightex_cam_settings_ui_vis(True)
            toggle_general_cam_settings_ui_vis(True)
        elif int(ui.cb_SystemSelection.currentIndex())==2:
            # Find the Boson Cameras
            #TODO: Figure out how to correctly connect two BOSONs.
            device_list = list_ports.comports()
            # Boson VID and PID:
            VID = 0x09CB
            PID = 0x4007
            port_list = []
            # Find all ports associated with Bosons.
            for device in device_list:
                if device.vid == VID and device.pid == PID:
                    port = device.device
                    port_list.append(port)
            cam_list = []
            device_id = None
            for boson_port in port_list:
                c = BosonCamera(port=boson_port, device_id=device_id)
                print(boson_port)
                ffc_state = 0
                c.do_ffc()
                while ffc_state == 0:
                    ffc_state = c.get_ffc_state()
                cam_list.append(c)
                cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
                device_id = 1
            toggle_mightex_cam_settings_ui_vis(False)
            toggle_general_cam_settings_ui_vis(True)
            toggle_BOSON_cam_settings_ui_vis(True)
        else:
            print("Choose a system!")

    def service_BOSON(cam_index):
        """
        Handle the FFCs and NUCs of Boson.
        #TODO: I should operate in manual still and correct this to use the right command.
          See the emails with FLIR, basically the command I was calling in flirpy was really giving a 0 or 1 not the 0-4
          output option
        """
        global cam_list, cam1_index, cam2_index

        if not (cam_index == cam1_index or cam_index==cam2_index):
            # If you are calling the function and not passing a cam_index that corresponds to 1 of the cameras, then
            # return without executing the function.
            return
        #Check if ffc is desired. If so, perform ffc
        if cam_list[cam_index].get_ffc_desired():
            cam_list[cam_index].do_ffc()
            num_ffc_performed = 1
            # Check to see if the FFC is complete. If not, wait until it is or initiate another FFC if requested.
            if cam_list[cam_index].get_ffc_state()!=3:
                switch = True
                while switch:
                    ffc_state = cam_list[cam_index].get_ffc_state()
                    print("FFC state:", ffc_state)
                    # 2 = FLR_BOSON_FFC_IN_PROGRESS
                    # 3 = FLR_BOSON_FFC_COMPLETE
                    if ffc_state == 3:
                        if cam_list[cam_index].get_ffc_desired():
                            cam_list[cam_index].do_ffc()
                            num_ffc_performed+=1
                            if num_ffc_performed>3:
                                print("The camera just did 3 successive ffcs.")
                        else:
                            switch = False
                    elif ffc_state == 2:
                        pass
                    elif ffc_state == 0:
                        cam_list[cam_index].do_ffc()
                    else:
                        print("The ffc state should be in progress or complete, but it is in neither. Is FFC Mode Manual?")
                        print(ffc_state)

        #Check if NUC table switch is desired. If so, switch NUC table.
        if cam_list[cam_index].get_nuc_desired():
            cam_list[cam_index].do_nuc_table_switch

        fpa_temp = cam_list[cam_index].get_fpa_temperature()
        if fpa_temp-68 > -5: # permanent damage to camera at as low as 68 C. Maybe 63 C is too conservative?
            while True:
                # TODO: Can I connect to an automatic shutter?
                print("SHUTTER THE BEAM IMMEDIATELY!!!")
                Cam1_LeftArrow.setVisible(True)
                Cam1_RightArrow.setVisible(True)
                Cam1_DownArrow.setVisible(True)
                Cam1_UpArrow.setVisible(True)
                Cam2_LeftArrow.setVisible(True)
                Cam2_RightArrow.setVisible(True)
                Cam2_DownArrow.setVisible(True)
                Cam2_UpArrow.setVisible(True)
        if cam_index == cam1_index:
            ui.label_17.setText(str(fpa_temp))
        elif cam_index == cam2_index:
            ui.label_19.setText(str(fpa_temp))

    def shut_down():
        global state
        pass
        """
        if int(ui.cb_SystemSelection.currentIndex()) == 1:
            IR = 0
        elif int(ui.cb_SystemSelection.currentIndex()) == 2:
            IR = 1
        UpdateManager.store_data(state = state, IR=IR)
        UpdateManager.reset_data()
        """
        return

    def capture_cam1_img():
        global cam1_index, cam1_threshold
        img, _, _, _, _ = take_img(cam1_index, cam_view=0, threshold=cam1_threshold,
                                                                 resetView=False)
        filename = "CameraImages/" + str(np.datetime64('now', 's')) + "_cam1.txt"
        np.savetxt(filename, img, fmt='%f')

    def capture_cam2_img():
        global cam2_index, cam2_threshold
        img, _, _, _, _ = take_img(cam2_index, cam_view=1, threshold=cam2_threshold,
                                   resetView=False)
        filename = "CameraImages/" + str(np.datetime64('now', 's')) + "_cam2.txt"
        np.savetxt(filename, img, fmt='%f')

    ui.btn_cam1_update.clicked.connect(update_cam1_settings)
    ui.btn_cam2_update.clicked.connect(update_cam2_settings)
    ui.btn_motor_connect.clicked.connect(update_motors)
    ui.act_calibrate.triggered.connect(begin_calibration)
    ui.actionLoad_Old_Home.triggered.connect(load_home)
    ui.btn_clear.clicked.connect(clear_pointing_plots)
    ui.btn_lock.clicked.connect(lock_pointing)
    ui.btn_Home.clicked.connect(define_home)
    ui.btn_Align.clicked.connect(begin_align)
    ui.pb_cam1_img_cap.clicked.connect(capture_cam1_img)
    ui.pb_cam2_img_cap.clicked.connect(capture_cam2_img)
    ui.cb_SystemSelection.currentIndexChanged.connect(find_cameras)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(UPDATE_TIME)
    MainWindow.show()

    sys.exit(app.exec_())
