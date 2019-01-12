class CamSettings():
    def __init__(self):
        self._exp_time = 0.05
        self._gain = 1
        self._threshold = 0
        self._decimate = False
    
    @property
    def gain(self):
        return self._gain
    
    @gain.setter
    def gain(self, value):
        value = int(round(value*8))
        if value < 1:
            value = 1
        if value > 64:
            value = 64
        self._gain = value
    
    @property
    def exp_time(self):
        return self._exp_time
    
    @exp_time.setter
    def exp_time(self, value):
        pass
        

if __name__ == "__main__":
    import sys
    import usb
    import time
    import numpy as np
    from pointing_ui import Ui_MainWindow
    from PyQt5 import QtCore, QtGui, QtWidgets
    from camera import Camera

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.tbl_devices.setColumnCount(2)
    cam_model = QtGui.QStandardItemModel()
    #ui.tbl_devices.setHorizontalHeaderItem(0, QtWidgets.QTableWidgetItem('Module No.'))
    #ui.tbl_devices.setHorizontalHeaderItem(1, QtWidgets.QTableWidgetItem('Serial No.'))

    cam_dev_list = list(usb.core.find(find_all=True, idVendor=0x04B4))  # list of cameras
    if len(cam_dev_list) == 0:
        print('Could not find any Mightex cameras!')
    else:
        ui.tbl_devices.setRowCount(len(cam_dev_list))
        cam_list = []
        for i, cam in enumerate(cam_dev_list):
            c = Camera(dev=cam)
            cam_list.append(c)
            cam_model.appendRow(QtGui.QStandardItem(c.serial_no))
            #ui.tbl_devices.setItem(i, 0, QtWidgets.QTableWidgetItem(c.module_no))
            #ui.tbl_devices.setItem(i, 1, QtWidgets.QTableWidgetItem(c.serial_no))
        ui.cb_cam1.setModel(cam_model)
        ui.cb_cam2.setModel(cam_model)

    cam1_index = -1
    cam2_index = -1

    def update():
        """
        GUI update function
        """
        # TODO: check for saturation
        # TODO: calculate center of mass
        # TODO: talk to piezo controller
        if cam1_index >= 0:
            img = cam_list[cam1_index].get_frame()
            ui.le_cam1_max.setText(str(np.max(img)))
            ui.gv_camera1.setImage(img)
        if cam2_index >= 0:
            img = cam_list[cam2_index].get_frame()
            ui.le_cam2_max.setText(str(np.max(img)))
            ui.gv_camera2.setImage(img)

    def update_cam1_settings():
        global cam1_index
        cam1_index = int(ui.cb_cam1.currentIndex())
        cam1_exp_time = float(ui.le_cam1_exp_time.text())
        print('cam1_exp_time', cam1_exp_time)
        cam1_gain = float(ui.le_cam1_gain.text())
        #cam1_threshold = float(ui.le_cam1_threshold.text())
        cam1_decimate = ui.cb_cam1_decimate.isChecked()
        cam_list[cam1_index].set_gain(cam1_gain)
        cam_list[cam1_index].set_exposure_time(cam1_exp_time)
        cam_list[cam1_index].set_decimation(cam1_decimate)
        ui.le_cam1_exp_time.setText('%.2f' % (cam_list[cam1_index].exposure_time))
        ui.le_cam1_gain.setText('%.2f' % (cam_list[cam1_index].gain/8))
    
    def update_cam2_settings():
        global cam2_index
        cam2_index = int(ui.cb_cam2.currentIndex())
        cam2_exp_time = float(ui.le_cam2_exp_time.text())
        cam2_gain = float(ui.le_cam2_gain.text())
        #cam2_threshold = float(ui.le_cam2_threshold.text())
        cam2_decimate = ui.cb_cam2_decimate.isChecked()
        cam_list[cam2_index].set_gain(cam2_gain)
        cam_list[cam2_index].set_exposure_time(cam2_exp_time)
        cam_list[cam2_index].set_decimation(cam2_decimate)
        ui.le_cam2_exp_time.setText('%.2f' % (cam_list[cam2_index].exposure_time))
        ui.le_cam2_gain.setText('%.2f' % (cam_list[cam2_index].gain/8))

    def conn_to_camera():
        global cam_index
        selection = ui.tbl_devices.selectedIndexes()
        if len(selection) > 0:
            cam_index = ui.tbl_devices.selectedIndexes()[0].row()

    ui.btn_cam1_update.clicked.connect(update_cam1_settings)
    ui.btn_cam2_update.clicked.connect(update_cam2_settings)

    ui.btn_connect.clicked.connect(conn_to_camera)
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(200)
    MainWindow.show()

    sys.exit(app.exec_())
