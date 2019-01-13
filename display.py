# TODO: stop using global vars and put this code into a fucking window

if __name__ == "__main__":
    import sys
    import usb
    import time
    import numpy as np
    import pyqtgraph as pg
    from pointing_ui import Ui_MainWindow
    from PyQt5 import QtCore, QtGui, QtWidgets
    from camera import Camera
    from camera import FakeCamera

    pg.setConfigOptions(imageAxisOrder='row-major')
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')

    UPDATE_TIME = 500  # ms

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    cam_model = QtGui.QStandardItemModel()

    cam_dev_list = list(usb.core.find(find_all=True, idVendor=0x04B4))  # list of cameras
    if len(cam_dev_list) == 0:
        print('Could not find any Mightex cameras!')
    else:
        cam_list = []
        for i, cam in enumerate(cam_dev_list):
            c = Camera(dev=cam)
            cam_list.append(c)
            cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
    # Add fake cameras
    for i in range(3):
        c = FakeCamera()
        cam_list.append(c)
        cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
    ui.cb_cam1.setModel(cam_model)
    ui.cb_cam2.setModel(cam_model)

    cam1_x = np.ndarray(0)
    cam1_y = np.ndarray(0)
    cam2_x = np.ndarray(0)
    cam2_y = np.ndarray(0)
    cam1_x_time = np.ndarray(0)
    cam1_y_time = np.ndarray(0)
    cam2_x_time = np.ndarray(0)
    cam2_y_time = np.ndarray(0)
    cam1_x_plot = ui.gv_cam_xy.addPlot(row=0, col=0).plot()
    cam1_y_plot = ui.gv_cam_xy.addPlot(row=1, col=0).plot()
    cam2_x_plot = ui.gv_cam_xy.addPlot(row=2, col=0).plot()
    cam2_y_plot = ui.gv_cam_xy.addPlot(row=3, col=0).plot()

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

    cam1_index = -1
    cam2_index = -1
    cam1_threshold = 0
    cam2_threshold = 0
    cam1_reset = True
    cam2_reset = True

    saturated_cam1 = False
    under_saturated_cam1 = False
    saturated_cam2 = False
    under_saturated_cam2 = False

    def resetHist(ref, min=0, max=255):
        assert isinstance(ref, pg.ImageView)
        ref.getHistogramWidget().setHistogramRange(min, max)
        ref.getHistogramWidget().setLevels(min, max)

    def calc_com(img):
        Nx, Ny = img.shape
        x = np.arange(Nx)
        y = np.arange(Ny)
        X, Y = np.meshgrid(x, y, indexing='ij')
        w = img/np.sum(img)
        com_x = np.sum(X*w)
        com_y = np.sum(Y*w)
        return com_x, com_y

    def update():
        """
        GUI update function
        """
        # TODO: check for saturation
        # TODO: calculate center of mass
        # TODO: talk to piezo controller
        start_time = time.time()
        global saturated_cam1, under_saturated_cam1, saturated_cam2, under_saturated_cam2
        global cam1_reset, cam2_reset
        global cam1_x, cam1_y, cam2_x, cam2_y
        global cam1_x_time, cam1_y_time, cam2_x_time, cam2_y_time
        # Camera 1 update
        if cam1_index >= 0:
            img = cam_list[cam1_index].get_frame()
            under_saturated_cam1 = np.all(img < 50)
            saturated_cam1 = np.any(img > 250)
            img[img < cam1_threshold] = 0
            ui.le_cam1_max.setText(str(np.max(img)))
            if not under_saturated_cam1 and not saturated_cam1:
                com_x, com_y = calc_com(img)
                if (cam1_x.size < 100):
                    cam1_x = np.append(cam1_x, com_x)
                    cam1_y = np.append(cam1_y, com_y)
                    #cam1_x_time = np.append(cam1_x, len(cam1_x))
                    #cam1_y_time = np.append(cam1_y, len(cam1_y))
                else:
                    cam1_x = np.roll(cam1_x, -1)
                    cam1_x[-1] = com_x
                    cam1_y = np.roll(cam1_y, -1)
                    cam1_y[-1] = com_y
                cam1_x_plot.setData(cam1_x)
                cam1_y_plot.setData(cam1_y)
            if cam1_reset:
                ui.gv_camera1.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False)
                cam1_reset = False
            else:
                ui.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
        # Camera 2 update
        if cam2_index >= 0:
            img = cam_list[cam2_index].get_frame()
            under_saturated_cam2 = np.all(img < 50)
            saturated_cam2 = np.any(img > 254)
            img[img < cam2_threshold] = 0
            ui.le_cam2_max.setText(str(np.max(img)))
            if not under_saturated_cam2 and not saturated_cam2:
                com_x, com_y = calc_com(img)
                if (cam2_x.size < 100):
                    cam2_x = np.append(cam2_x, com_x)
                    cam2_y = np.append(cam2_y, com_y)
                    #cam2_x_time = np.append(cam2_x, len(cam2_x))
                    #cam2_y_time = np.append(cam2_y, len(cam2_y))
                else:
                    cam2_x = np.roll(cam2_x, -1)
                    cam2_x[-1] = com_x
                    cam2_y = np.roll(cam2_y, -1)
                    cam2_y[-1] = com_y
                cam2_x_plot.setData(cam2_x)
                cam2_y_plot.setData(cam2_y)
            if cam2_reset:
                ui.gv_camera2.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False)
                cam2_reset = False
            else:
                ui.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
        ui.statusbar.showMessage('Update time: {:.3f} (s)'.format(time.time() - start_time))

    def update_cam1_settings():
        global cam1_index, cam1_threshold, cam1_reset
        cam1_index = int(ui.cb_cam1.currentIndex())
        cam1_exp_time = float(ui.le_cam1_exp_time.text())
        cam1_gain = float(ui.le_cam1_gain.text())
        cam1_threshold = float(ui.le_cam1_threshold.text())
        cam1_decimate = ui.cb_cam1_decimate.isChecked()
        cam_list[cam1_index].set_gain(cam1_gain)
        cam_list[cam1_index].set_exposure_time(cam1_exp_time)
        cam_list[cam1_index].set_decimation(cam1_decimate)
        ui.le_cam1_exp_time.setText('%.2f' % (cam_list[cam1_index].exposure_time))
        ui.le_cam1_gain.setText('%.2f' % (cam_list[cam1_index].gain/8))
        cam1_reset = True
        resetHist(ui.gv_camera1)
    
    def update_cam2_settings():
        global cam2_index, cam2_threshold, cam2_reset
        cam2_index = int(ui.cb_cam2.currentIndex())
        cam2_exp_time = float(ui.le_cam2_exp_time.text())
        cam2_gain = float(ui.le_cam2_gain.text())
        cam2_threshold = float(ui.le_cam2_threshold.text())
        cam2_decimate = ui.cb_cam2_decimate.isChecked()
        cam_list[cam2_index].set_gain(cam2_gain)
        cam_list[cam2_index].set_exposure_time(cam2_exp_time)
        cam_list[cam2_index].set_decimation(cam2_decimate)
        ui.le_cam2_exp_time.setText('%.2f' % (cam_list[cam2_index].exposure_time))
        ui.le_cam2_gain.setText('%.2f' % (cam_list[cam2_index].gain/8))
        cam2_reset = True
        resetHist(ui.gv_camera2)

    ui.btn_cam1_update.clicked.connect(update_cam1_settings)
    ui.btn_cam2_update.clicked.connect(update_cam2_settings)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(UPDATE_TIME)
    MainWindow.show()

    sys.exit(app.exec_())
