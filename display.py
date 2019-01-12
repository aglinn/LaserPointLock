if __name__ == "__main__":
    import sys
    import usb
    import time
    from pointing_ui import Ui_MainWindow
    from PyQt5 import QtCore, QtGui, QtWidgets
    from camera import Camera

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.tbl_devices.setColumnCount(2)
    ui.tbl_devices.setHorizontalHeaderItem(0, QtWidgets.QTableWidgetItem('Module No.'))
    ui.tbl_devices.setHorizontalHeaderItem(1, QtWidgets.QTableWidgetItem('Serial No.'))

    cam_dev_list = list(usb.core.find(find_all=True, idVendor=0x04B4))  # list of cameras
    if len(cam_dev_list) == 0:
        print('Could not find any Mightex cameras!')
    else:
        ui.tbl_devices.setRowCount(len(cam_dev_list))
        cam_list = []
        for i, cam in enumerate(cam_dev_list):
            c = Camera(dev=cam)
            cam_list.append(c)
            ui.tbl_devices.setItem(i, 0, QtWidgets.QTableWidgetItem(c.module_no))
            ui.tbl_devices.setItem(i, 1, QtWidgets.QTableWidgetItem(c.serial_no))
            #QtWidgets.QTableWidgetItem(c.serial_no)
    #ui.centralwidget.setFixedSize(ui.horizontalLayout.sizeHint())

    cam_index = -1

    def update():
        global cam_index
        if cam_index < 0:
            return
        else:
            img = cam_list[cam_index].get_frame()
            ui.gv_camera1.setImage(img)

    def conn_to_camera():
        global cam_index
        selection = ui.tbl_devices.selectedIndexes()
        if len(selection) > 0:
            cam_index = ui.tbl_devices.selectedIndexes()[0].row()

    ui.btn_connect.clicked.connect(conn_to_camera)
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(200)
    MainWindow.show()

    sys.exit(app.exec_())
