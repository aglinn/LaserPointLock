def calc_com_x(img):
    Nx, Ny = img.shape
    x = np.arange(Nx)
    y = np.arange(Ny)
    X, _ = np.meshgrid(x, y, indexing='ij')
    w = img / np.sum(img)
    com_x = np.sum(X * w)
    return com_x


def calc_com_y(img):
    Nx, Ny = img.shape
    x = np.arange(Nx)
    y = np.arange(Ny)
    _, Y = np.meshgrid(x, y, indexing='ij')
    w = img / np.sum(img)
    com_y = np.sum(Y * w)
    return com_y


def calc_com(img, cam_view=None, cam_index=[]):
    """
    Given an image, img, calculate the center of mass in pixel coordinates
    """
    Nx, Ny = img.shape
    x = np.arange(Nx)
    y = np.arange(Ny)
    X, Y = np.meshgrid(x, y, indexing='ij')
    try:
        w = img / np.sum(img)
    except RuntimeWarning:
        # If dividing by 0, then the frame is empty. So return set position as COM.
        if cam_view == 0:
            x0, y0 = cam_list[cam_index].startXY
            return UpdateManager.set_pos[0:2] - np.asarray([y0, x0])
        elif cam_view == 1:
            x0, y0 = cam_list[cam_index].startXY
            return UpdateManager.set_pos[2:] - np.asarray([y0, x0])
        else:
            return np.floor(Nx / 2), np.floor(Ny / 2)
    com_x = np.sum(X * w)
    com_y = np.sum(Y * w)
    return com_x, com_y


def calc_com_opt(img):
    """Given an img, calculate the com in pixel coordinates. Algorithm slightly
    optimized by 1) calculating the maximum, 2) estimating the width by finding
    the max/4 position, and 3) cropping the window to the max/4 position and
    calculating com
    """
    i, j = np.unravel_index(np.argmax(img), img.shape)
    r = np.abs(i - np.argmin(np.abs(img[i, j] / 4 - img[:, j])))
    start_i = max(0, i - r)
    end_i = min(img.shape[0] - 1, i + r)
    start_j = max(0, j - r)
    end_j = min(img.shape[1] - 1, j + r)
    com_x, com_y = calc_com(img[start_i:end_i, start_j:end_j])
    return com_x + start_i, com_y + start_j

    def take_img_calibration(self, cam_index, cam_view=0, threshold=0):
        global state
        global cam1_x, cam2_x, cam1_y, cam2_y
        global cam1_x_line, cam2_x_line, cam1_y_line, cam2_y_line
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        if int(self.cb_SystemSelection.currentIndex()) == 2:
            service_BOSON(cam_index)
        if cam_view == 0:
            avg_frames = int(self.Cam1_AvgFrames.text())
        else:
            avg_frames = int(self.Cam2_AvgFrames.text())
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
            img = img / avg_frames
        if threshold > 0:
            img[img < threshold * avg_frames] = 0
        if cam_view == 0:
           self.le_cam1_max.setText(
                str(np.max(img / avg_frames)))  # Just updates the GUI to say what the max value of the camera is
        else:
           self.le_cam2_max.setText(
                str(np.max(img / avg_frames)))  # Just updates the GUI to say what the max value of the camera is
        com = self.calc_com(img)  # Grab COM

        # The below just plots the last 100 points of the COM for x and y
        if cam_view == 0:  # Decide whether cam 0 or 1, which camera?
            # x
            cam1_x_line.setPos(com[0])
            cam1_x_line.setVisible(True)
            if (cam1_x.size < int(self.le_num_points.text())):
                cam1_x = np.append(cam1_x, com[0])
            else:
                cam1_x = np.roll(cam1_x, -1)
                cam1_x[-1] = com[0]
            cam1_x_plot.setData(cam1_x)
            # y
            cam1_y_line.setPos(com[1])
            cam1_y_line.setVisible(True)
            if (cam1_y.size < int(self.le_num_points.text())):
                cam1_y = np.append(cam1_y, com[1])
            else:
                cam1_y = np.roll(cam1_y, -1)
                cam1_y[-1] = com[1]
            cam1_y_plot.setData(cam1_y)
        # Last 100 points of COM have been plotted, removing old points to maintain just 100.
        # Ditto the above for camera 2 below
        else:
            # x
            cam2_x_line.setPos(com[0])
            cam2_x_line.setVisible(True)
            if (cam2_x.size < int(self.le_num_points.text())):
                cam2_x = np.append(cam2_x, com[0])
            else:
                cam2_x = np.roll(cam2_x, -1)
                cam2_x[-1] = com[0]
            cam2_x_plot.setData(cam2_x)
            # y
            cam2_y_line.setPos(com[1])
            cam2_y_line.setVisible(True)
            if (cam2_y.size < int(self.le_num_points.text())):
                cam2_y = np.append(cam2_y, com[1])
            else:
                cam2_y = np.roll(cam2_y, -1)
                cam2_y[-1] = com[1]
            cam2_y_plot.setData(cam2_y)
        return img, com

    def update(self):
        global LockTimeStart
        global motor_list
        global msg, most_recent_error
        global start_time
        start_time = time.time()
        msg = ''
        if state == self.STATE_MEASURE:
            updateMeasure()
            msg += 'UNLOCKED'
        elif state == self.STATE_CALIBRATE:
            updateCalibrate()
            msg += 'Calibrating...'
        elif state == self.STATE_LOCKED:
            updateLocked()
            msg += 'LOCKED'
        elif state == self.STATE_ALIGN:
            updateAlign()
            msg += 'Alignment Mode'
        else:
            msg += 'ERROR'
        self.statusbar.showMessage(
            '{0}\tUpdate time: {1:.3f} (s)'.format(msg, time.time() - start_time) + most_recent_error)
        gc.collect()
        return

    def take_img(self, cam_index, cam_view=0, threshold=0, resetView=False):
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
        global cam1_threshold, cam2_threshold
        assert cam_index >= 0
        assert cam_view == 0 or cam_view == 1
        global msg

        if int(self.cb_SystemSelection.currentIndex()) == 2:
            service_BOSON(cam_index)
        if cam_view == 0:
            avg_frames = int(self.Cam1_AvgFrames.text())
        else:
            avg_frames = int(self.Cam2_AvgFrames.text())
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
            img = img / avg_frames
        if threshold > 0:
            if np.any(img > threshold):
                img[img < threshold * avg_frames] = 0
            else:
                msg += 'Threshold too high = not applied.'

        if cam_view == 0:
           self.le_cam1_max.setText(
                str(np.max(img / avg_frames)))  # Just updates the GUI to say what the max value of the camera is
            if cam1_index >= 0 and cam2_index >= 0:
                UpdateManager.cam_1_img = img
                UpdateManager.t1 = cam_list[cam_index].time
        else:
           self.le_cam2_max.setText(
                str(np.max(img / avg_frames)))  # Just updates the GUI to say what the max value of the camera is
            if cam1_index >= 0 and cam2_index >= 0:
                UpdateManager.cam_2_img = img
                UpdateManager.t2 = cam_list[cam_index].time
        com = self.calc_com(img, cam_view, cam_index)  # Grab COM
        x0, y0 = cam_list[cam_index].startXY
        com = (com[0] + y0, com[1] + x0)
        under_saturated = np.all(img < 50)
        saturated = np.any(img > 250)
        if cam_view == 0:
            UpdateManager.cam_1_com = np.asarray(com)
           self.le_cam1_max.setText(str(np.max(img)))
            cam_x_line = cam1_x_line
            cam_y_line = cam1_y_line
            cam_x_data = cam1_x
            cam_y_data = cam1_y
            cam_x_plot = cam1_x_plot
            cam_y_plot = cam1_y_plot
            gv_camera =self.gv_camera1
        else:
            UpdateManager.cam_2_com = np.asarray(com)
           self.le_cam2_max.setText(str(np.max(img)))
            cam_x_line = cam2_x_line
            cam_y_line = cam2_y_line
            cam_x_data = cam2_x
            cam_y_data = cam2_y
            cam_x_plot = cam2_x_plot
            cam_y_plot = cam2_y_plot
            gv_camera =self.gv_camera2
        # Calculate COM
        com_x = com[0]
        com_y = com[1]
        cam_x_line.setPos(com_x)
        cam_y_line.setPos(com_y)
        cam_x_line.setVisible(True)
        cam_y_line.setVisible(True)
        if (cam_x_data.size < int(self.le_num_points.text())):
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
        if notself.cb_suppress_pointing_updat.isChecked():
            cam_x_plot.setData(cam_x_data)
            cam_y_plot.setData(cam_y_data)

        if notself.cb_suppress_image_update.isChecked():
            if resetView:
                gv_camera.setImage(img, autoRange=True, autoLevels=False, autoHistogramRange=False, pos=(x0, y0))
            else:
                gv_camera.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False, pos=(x0, y0))
        return img, com_x, com_y, under_saturated, saturated

    def updateCalibrate(self):
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
        global reset_piezo  # Switch variable.
        if (self.cb_motors_1.currentIndex() < 0):
            error_dialog.showMessage('You need to select Motor 1.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        elif (self.cb_motors_2.currentIndex() < 0):
            error_dialog.showMessage('You need to select Motor 2.')
            shut_down()
            state = STATE_MEASURE
            ROICam1_Lock.setVisible(False)
            ROICam2_Lock.setVisible(False)
            ROICam1_Unlock.setVisible(True)
            ROICam2_Unlock.setVisible(True)
            return
        elifself.cb_motors_1.currentIndex() ==self.cb_motors_2.currentIndex():
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
                print(motor_index, voltage_step, set_voltage)
                if voltage_step == 0:
                    if motor_index == 1:
                        motor_list[0].ch1_v = starting_v[0]
                        time.sleep(0.5)
                    elif motor_index == 2:
                        motor_list[0].ch2_v = starting_v[1]
                        time.sleep(0.5)
                    elif motor_index == 3:
                        motor_list[1].ch1_v = starting_v[2]
                        time.sleep(0.5)
                if motor_index == 0:
                    # first motor, channel 1
                    motor_list[0].ch1_v = set_voltage
                    if voltage_step == 0:
                        time.sleep(5.0)
                    else:
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                    voltage = motor_list[0].ch1_v
                    time.sleep(0.1)
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        override = False
                        reset_piezo = 0
                        # update motor voltages
                        motor1_x = addToPlot(motor1_x, motor1_x_plot, voltage)
                        # get a frame from cam1 for x motor
                        img, com = take_img_calibration(cam1_index, cam_view=0, threshold=cam1_threshold)
                        # put in list, update image
                        mot1_x_cam1_x[voltage_step] = com[0]
                        mot1_x_cam1_y[voltage_step] = com[1]
                       self.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2 for x motor
                        img, com = take_img_calibration(cam2_index, cam_view=1, threshold=cam2_threshold)
                        # put in list
                        mot1_x_cam2_x[voltage_step] = com[0]
                        mot1_x_cam2_y[voltage_step] = com[1]
                       self.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot1_x_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        # If the voltage is not where it should be, update it.
                        motor_list[0].ch1_v = set_voltage
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                        if reset_piezo > 0:  # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
                elif motor_index == 1:
                    # first motor, channel 2
                    motor_list[0].ch2_v = set_voltage
                    if voltage_step == 0:
                        time.sleep(5.0)
                    else:
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                    voltage = motor_list[0].ch2_v
                    time.sleep(0.1)
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        override = False
                        reset_piezo = 0
                        # update motor voltages
                        motor1_y = addToPlot(motor1_y, motor1_y_plot, voltage)
                        # get a frame from cam1
                        # img, com = take_img_calibration(cam1_index, cam_view = 0, threshold = cam1_threshold, avg_frames = 2, avg_com = 2)
                        img, com = take_img_calibration(cam1_index, cam_view=0, threshold=cam1_threshold)
                        # put in list
                        mot1_y_cam1_x[voltage_step] = com[0]
                        mot1_y_cam1_y[voltage_step] = com[1]
                       self.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2
                        img, com = take_img_calibration(cam2_index, cam_view=1, threshold=cam2_threshold)
                        # put in list
                        mot1_y_cam2_x[voltage_step] = com[0]
                        mot1_y_cam2_y[voltage_step] = com[1]
                       self.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot1_y_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        # If the voltage is not where it should be, update it.
                        motor_list[0].ch2_v = set_voltage
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                        if reset_piezo > 0:  # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
                elif motor_index == 2:
                    # second motor, channel 1
                    motor_list[1].ch1_v = set_voltage
                    if voltage_step == 0:
                        time.sleep(5.0)
                    else:
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                    voltage = motor_list[1].ch1_v
                    time.sleep(0.1)
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        reset_piezo = 0
                        override = False
                        # update motor voltages
                        motor2_x = addToPlot(motor2_x, motor2_x_plot, voltage)
                        # get a frame from cam1 for x motor
                        img, com = take_img_calibration(cam1_index, cam_view=0, threshold=cam1_threshold)
                        # put in list, update image
                        mot2_x_cam1_x[voltage_step] = com[0]
                        mot2_x_cam1_y[voltage_step] = com[1]
                       self.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2 for x motor
                        img, com = take_img_calibration(cam2_index, cam_view=1, threshold=cam2_threshold)
                        # put in list
                        mot2_x_cam2_x[voltage_step] = com[0]
                        mot2_x_cam2_y[voltage_step] = com[1]
                       self.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # And save a list of the actual voltages
                        mot2_x_voltage[voltage_step] = voltage
                        # next step of calibration
                        calib_index += 1
                    else:
                        # If the voltage is not where it should be, update it.
                        motor_list[1].ch1_v = set_voltage
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                        if reset_piezo > 0:  # Fuck it overide the tolerance, because now the code is improved to store the actual voltage
                            override = True
                        reset_piezo += 1
                elif motor_index == 3:
                    # second motor, channel 2
                    motor_list[1].ch2_v = set_voltage
                    if voltage_step == 0:
                        time.sleep(5.0)
                    else:
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                    voltage = motor_list[1].ch2_v
                    time.sleep(0.1)
                    if (abs(voltage - set_voltage) < 0.2 or override):
                        time.sleep(0.2)  # Give time between setting the piezo and measuring the COM
                        reset_piezo = 0
                        override = False
                        # update motor voltages
                        motor2_y = addToPlot(motor2_y, motor2_y_plot, voltage)
                        # get a frame from cam1
                        img, com = take_img_calibration(cam1_index, cam_view=0, threshold=cam1_threshold)
                        # put in list
                        mot2_y_cam1_x[voltage_step] = com[0]
                        mot2_y_cam1_y[voltage_step] = com[1]
                       self.gv_camera1.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
                        # get a frame from cam2
                        img, com = take_img_calibration(cam2_index, cam_view=1, threshold=cam2_threshold)
                        # put in list
                        mot2_y_cam2_x[voltage_step] = com[0]
                        mot2_y_cam2_y[voltage_step] = com[1]
                       self.gv_camera2.setImage(img, autoRange=False, autoLevels=False, autoHistogramRange=False)
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
                time.sleep(0.2)
                """if int(self.cb_SystemSelection.currentIndex()) == 1:
                    # Put the Mightex cameras into normal mode--No trigger required.
                    for cam in cam_list:
                        cam.engine.update_working_mode(0, cam.serial_no)"""
                """
                The cameras suck. They are returning frames in an asynchronous fashion. Forcing the cameras to
                wait for a trigger to capture a frame causes frame grab to take ~2 seconds, rediculous. Unfortunately,
                I cannot figure out exactly when the frames are coming from in continuous mode, but it is clear that often
                the frames are coming from before the last piezzo update, which is obviously very bad. We probably will
                get better cameras, but for now, I want to try to find a work around...
                So, I will increase the number of calibration points, but I will delete the first 2 and last 2 points in
                each data series before doing a polynomial fit to avoid very bad fits. At least, this should give me a 
                good calibration. The above problem is still not good for the locking part of the algorithm, but maybe 
                it can be good enough to keep my pointing generally locked for alignment purposes. I need new cameras 
                before data acquisition begins. 
                """
                mot1_x_voltage = mot1_x_voltage[2:-2]
                mot1_y_voltage = mot1_y_voltage[2:-2]
                mot2_x_voltage = mot2_x_voltage[2:-2]
                mot2_y_voltage = mot2_y_voltage[2:-2]

                mot1_x_cam1_x = mot1_x_cam1_x[2:-2]
                mot1_x_cam1_y = mot1_x_cam1_y[2:-2]
                mot1_x_cam2_x = mot1_x_cam2_x[2:-2]
                mot1_x_cam2_y = mot1_x_cam2_y[2:-2]

                mot1_y_cam1_x = mot1_y_cam1_x[2:-2]
                mot1_y_cam1_y = mot1_y_cam1_y[2:-2]
                mot1_y_cam2_x = mot1_y_cam2_x[2:-2]
                mot1_y_cam2_y = mot1_y_cam2_y[2:-2]

                mot2_x_cam1_x = mot2_x_cam1_x[2:-2]
                mot2_x_cam1_y = mot2_x_cam1_y[2:-2]
                mot2_x_cam2_x = mot2_x_cam2_x[2:-2]
                mot2_x_cam2_y = mot2_x_cam2_y[2:-2]

                mot2_y_cam1_x = mot2_y_cam1_x[2:-2]
                mot2_y_cam1_y = mot2_y_cam1_y[2:-2]
                mot2_y_cam2_x = mot2_y_cam2_x[2:-2]
                mot2_y_cam2_y = mot2_y_cam2_y[2:-2]

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

                def plot_above_polyfit_results():
                    Voltage_plot = np.linspace(0, 150, 100)
                    fig, ax = plt.subplots(4, 4, dpi=200, gridspec_kw={"hspace": 0.5, "wspace": 0.3})
                    ax[0, 0].plot(Voltage_plot, p_mot1_x_cam1_x[0] * Voltage_plot + p_mot1_x_cam1_x[1])
                    ax[1, 0].plot(Voltage_plot, p_mot1_y_cam1_x[0] * Voltage_plot + p_mot1_y_cam1_x[1])
                    ax[2, 0].plot(Voltage_plot, p_mot2_x_cam1_x[0] * Voltage_plot + p_mot2_x_cam1_x[1])
                    ax[3, 0].plot(Voltage_plot, p_mot2_y_cam1_x[0] * Voltage_plot + p_mot2_y_cam1_x[1])
                    ax[0, 1].plot(Voltage_plot, p_mot1_x_cam1_y[0] * Voltage_plot + p_mot1_x_cam1_y[1])
                    ax[1, 1].plot(Voltage_plot, p_mot1_y_cam1_y[0] * Voltage_plot + p_mot1_y_cam1_y[1])
                    ax[2, 1].plot(Voltage_plot, p_mot2_x_cam1_y[0] * Voltage_plot + p_mot2_x_cam1_y[1])
                    ax[3, 1].plot(Voltage_plot, p_mot2_y_cam1_y[0] * Voltage_plot + p_mot2_y_cam1_y[1])
                    ax[0, 2].plot(Voltage_plot, p_mot1_x_cam2_x[0] * Voltage_plot + p_mot1_x_cam2_x[1])
                    ax[1, 2].plot(Voltage_plot, p_mot1_y_cam2_x[0] * Voltage_plot + p_mot1_y_cam2_x[1])
                    ax[2, 2].plot(Voltage_plot, p_mot2_x_cam2_x[0] * Voltage_plot + p_mot2_x_cam2_x[1])
                    ax[3, 2].plot(Voltage_plot, p_mot2_y_cam2_x[0] * Voltage_plot + p_mot2_y_cam2_x[1])
                    ax[0, 3].plot(Voltage_plot, p_mot1_x_cam2_y[0] * Voltage_plot + p_mot1_x_cam2_y[1])
                    ax[1, 3].plot(Voltage_plot, p_mot1_y_cam2_y[0] * Voltage_plot + p_mot1_y_cam2_y[1])
                    ax[2, 3].plot(Voltage_plot, p_mot2_x_cam2_y[0] * Voltage_plot + p_mot2_x_cam2_y[1])
                    ax[3, 3].plot(Voltage_plot, p_mot2_y_cam2_y[0] * Voltage_plot + p_mot2_y_cam2_y[1])

                    ax[0, 0].plot(mot1_x_voltage, mot1_x_cam1_x, 'r', marker='x')
                    ax[1, 0].plot(mot1_y_voltage, mot1_y_cam1_x, 'r', marker='x')
                    ax[2, 0].plot(mot2_x_voltage, mot2_x_cam1_x, 'r', marker='x')
                    ax[3, 0].plot(mot2_y_voltage, mot2_y_cam1_x, 'r', marker='x')
                    ax[0, 1].plot(mot1_x_voltage, mot1_x_cam1_y, 'r', marker='x')
                    ax[1, 1].plot(mot1_y_voltage, mot1_y_cam1_y, 'r', marker='x')
                    ax[2, 1].plot(mot2_x_voltage, mot2_x_cam1_y, 'r', marker='x')
                    ax[3, 1].plot(mot2_y_voltage, mot2_y_cam1_y, 'r', marker='x')
                    ax[0, 2].plot(mot1_x_voltage, mot1_x_cam2_x, 'r', marker='x')
                    ax[1, 2].plot(mot1_y_voltage, mot1_y_cam2_x, 'r', marker='x')
                    ax[2, 2].plot(mot2_x_voltage, mot2_x_cam2_x, 'r', marker='x')
                    ax[3, 2].plot(mot2_y_voltage, mot2_y_cam2_x, 'r', marker='x')
                    ax[0, 3].plot(mot1_x_voltage, mot1_x_cam2_y, 'r', marker='x')
                    ax[1, 3].plot(mot1_y_voltage, mot1_y_cam2_y, 'r', marker='x')
                    ax[2, 3].plot(mot2_x_voltage, mot2_x_cam2_y, 'r', marker='x')
                    ax[3, 3].plot(mot2_y_voltage, mot2_y_cam2_y, 'r', marker='x')

                    ax[0, 0].set_title("mot 1 x cam 1 x", fontsize=6)
                    ax[1, 0].set_title("mot 1 y cam 1 x", fontsize=6)
                    ax[2, 0].set_title("mot 2 x cam 1 x", fontsize=6)
                    ax[3, 0].set_title("mot 2 y cam 1 x", fontsize=6)
                    ax[0, 1].set_title("mot 1 x cam 1 y", fontsize=6)
                    ax[1, 1].set_title("mot 1 y cam 1 y", fontsize=6)
                    ax[2, 1].set_title("mot 2 x cam 1 y", fontsize=6)
                    ax[3, 1].set_title("mot 2 y cam 1 y", fontsize=6)
                    ax[0, 2].set_title("mot 1 x cam 2 x", fontsize=6)
                    ax[1, 2].set_title("mot 1 y cam 2 x", fontsize=6)
                    ax[2, 2].set_title("mot 2 x cam 2 x", fontsize=6)
                    ax[3, 2].set_title("mot 2 y cam 2 x", fontsize=6)
                    ax[0, 3].set_title("mot 1 x cam 2 y", fontsize=6)
                    ax[1, 3].set_title("mot 1 y cam 2 y", fontsize=6)
                    ax[2, 3].set_title("mot 2 x cam 2 y", fontsize=6)
                    ax[3, 3].set_title("mot 2 y cam 2 y", fontsize=6)
                    ax[3, 0].set_xlabel('Voltages (V)', fontsize=6)
                    ax[3, 0].set_ylabel('position (pixels)', fontsize=6)

                    ax[0, 0].tick_params(axis='both', which='major', labelsize=6)
                    ax[1, 0].tick_params(axis='both', which='major', labelsize=6)
                    ax[2, 0].tick_params(axis='both', which='major', labelsize=6)
                    ax[3, 0].tick_params(axis='both', which='major', labelsize=6)
                    ax[0, 1].tick_params(axis='both', which='major', labelsize=6)
                    ax[1, 1].tick_params(axis='both', which='major', labelsize=6)
                    ax[2, 1].tick_params(axis='both', which='major', labelsize=6)
                    ax[3, 1].tick_params(axis='both', which='major', labelsize=6)
                    ax[0, 2].tick_params(axis='both', which='major', labelsize=6)
                    ax[1, 2].tick_params(axis='both', which='major', labelsize=6)
                    ax[2, 2].tick_params(axis='both', which='major', labelsize=6)
                    ax[3, 2].tick_params(axis='both', which='major', labelsize=6)
                    ax[0, 3].tick_params(axis='both', which='major', labelsize=6)
                    ax[1, 3].tick_params(axis='both', which='major', labelsize=6)
                    ax[2, 3].tick_params(axis='both', which='major', labelsize=6)
                    ax[3, 3].tick_params(axis='both', which='major', labelsize=6)

                    plt.show()
                    return

                plot_above_polyfit_results()

                '''
                construct calibration matrix:
                Understand that this matrix is dV_i/dx_j where ij indexes like a normal matrix, i.e. row column. 
                Therefore, dV = calib_mat * dx where dx is a column vector, [d_cam1_x, d_cam1_y, d_cam2_x, d_cam2_y].Transpose,
                where, for example, d_cam1_x is the difference in the desired position of cam1_x and the calculated COM x_coordinate of cam 1.
                and dV is a column vector, [d_mot1_x, d_mot1_y, d_mot2_x, d_mot2_y,].Transpose, i.e. the change in voltage
                 on each motor to bring the COM coordinates to desired position.
                Therefore, this matrix can be used to update the motor voltages as New_V = Old_V - calib_mat*dx 
                '''
                calib_mat = np.array([[p_mot1_x_cam1_x[0], p_mot1_y_cam1_x[0], p_mot2_x_cam1_x[0], p_mot2_y_cam1_x[0]],
                                      [p_mot1_x_cam1_y[0], p_mot1_y_cam1_y[0], p_mot2_x_cam1_y[0], p_mot2_y_cam1_y[0]],
                                      [p_mot1_x_cam2_x[0], p_mot1_y_cam2_x[0], p_mot2_x_cam2_x[0], p_mot2_y_cam2_x[0]],
                                      [p_mot1_x_cam2_y[0], p_mot1_y_cam2_y[0], p_mot2_x_cam2_y[0], p_mot2_y_cam2_y[0]]])
                calib_mat = np.linalg.inv(calib_mat)
                UpdateManager.calibration_matrix = calib_mat
                try:
                    old_calib_mat = np.loadtxt('Most_Recent_Calibration.txt', dtype=float)
                    Change = np.sqrt(np.sum(np.square(calib_mat - old_calib_mat)))
                    RelativeChange = Change / np.sqrt(np.sum(np.square(old_calib_mat)))
                    print(RelativeChange)
                except OSError:
                    pass
                print('Calibration done!')
                print(calib_mat)
                if int(self.cb_SystemSelection.currentIndex()) == 1:
                    # Mightex cameras
                    np.savetxt('Most_Recent_Calibration.txt', calib_mat, fmt='%f')
                    filename = "CalibrationMatrixStored/" + str(np.datetime64('today', 'D')) + "_Calib_mat"
                    np.savetxt(filename, calib_mat, fmt='%f')
                elif int(self.cb_SystemSelection.currentIndex()) == 2:
                    # Boson cameras
                    np.savetxt('Most_Recent_Calibration_IR.txt', calib_mat, fmt='%f')
                    filename = "CalibrationMatrixStored/" + str(np.datetime64('today', 'D')) + "_Calib_mat_IR"
                    np.savetxt(filename, calib_mat, fmt='%f')
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)


    def updateMeasure(self):
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
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0,
                                                                         threshold=cam1_threshold, resetView=True)
                cam1_reset = False
            else:
                _, _, _, under_saturated_cam1, saturated_cam1 = take_img(cam1_index, cam_view=0,
                                                                         threshold=cam1_threshold, resetView=False)
        else:
            cam1_x_line.setVisible(False)
            cam1_y_line.setVisible(False)
        ############################
        # Camera 2 update function #
        ############################
        if cam2_index >= 0:
            if cam2_reset:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1,
                                                                         threshold=cam2_threshold, resetView=True)
                cam2_reset = False
            else:
                _, _, _, under_saturated_cam2, saturated_cam2 = take_img(cam2_index, cam_view=1,
                                                                         threshold=cam2_threshold, resetView=False)
        else:
            cam2_x_line.setVisible(False)
            cam2_y_line.setVisible(False)
        # Update dx with update manager if 2 cameras are connected:
        if cam1_index >= 0 and cam2_index >= 0:
            UpdateManager.calc_dx()
            GUI_update_std(UpdateManager.standard_deviation)


    def updateLocked(self):
        global ROICam1_Unlock, ROICam2_Unlock, ROICam1_Lock, ROICam2_Lock
        global motor1_x, motor1_y, motor2_x, motor2_y
        global motor1_x_plot, motor1_y_plot, motor2_x_plot, motor2_y_plot
        global LockTimeStart, TimeLastUnlock, time_unlocked
        global num_out_of_voltage_range  # Track the number of times the voltage update goes out of range during lock
        global state
        global first_unlock, Unlocked_report, Unlock_counter
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
                # print(update_voltage)
            except update_out_of_bounds:
                # This section of code keeps the update voltage in bounds, by setting anything out of bounds to the
                # limit of the bounds. Additionally, it tracks the number of times that the voltages went out of
                # bounds in less than 1 minute since the last out of bounds (i.e. if it goes out of bounds once but
                # then comes back in bounds for more than 1 minute, the counter is reset to 0). If the piezos go out
                # of bounds more than 10 times in a row in less than a minute each time, then the code unlocks and
                # returns to measuring state.

                # print("Warning: The update voltages went out of range")
                """for i in range(4):
                    if update_voltage[i] < 0:
                        update_voltage[i] = 0
                    if update_voltage[i] > 150:
                        update_voltage[i] = 150.0"""
                update_voltage = UpdateManager.fit_update()

                # Track time since last unlock and the number of times unlocked in less than 1 minute since previous
                # lock.
                if time.monotonic() - TimeLastUnlock < 60.0 and not first_unlock:
                    num_out_of_voltage_range += 1
                    TimeLastUnlock = time.monotonic()
                    Unlocked_report["round " + str(Unlock_counter) +
                                    " of going out of piezzo range."]["number of successive out of bounds"
                                                                      " updates with less than 60 seconds "
                                                                      "between updates"] = num_out_of_voltage_range
                    Unlocked_report["round " + str(Unlock_counter) +
                                    " of going out of piezzo range."]["Amount of time spent outside voltage "
                                                                      "range"] = time.monotonic() - time_unlocked
                    if np.linalg.norm(UpdateManager.dx[-1]) > Unlocked_report["round " + str(Unlock_counter) +
                                                                              " of going out of piezzo range."][
                        "Farthest dx during this round of unlock"]:
                        Unlocked_report["round " + str(Unlock_counter) +
                                        " of going out of piezzo range."][
                            "Farthest dx during this round of unlock"] = np.linalg.norm(UpdateManager.dx[-1])
                else:
                    first_unlock = False
                    TimeLastUnlock = time.monotonic()
                    num_out_of_voltage_range = 1
                    time_unlocked = time.monotonic()
                    Unlock_counter += 1
                    Unlocked_report["round " + str(Unlock_counter) +
                                    " of going out of piezzo range."] = {"number of successive out of bounds"
                                                                         " updates with less than 60 seconds "
                                                                         "between updates": num_out_of_voltage_range,
                                                                         "Amount of time spent outside voltage "
                                                                         "range": 0,
                                                                         "Farthest dx during this round of unlock":
                                                                             np.linalg.norm(UpdateManager.dx[-1])}
                   self.list_unlock_report.addItem("round " + str(Unlock_counter) +
                                                  " of going out of piezzo range.")
                if not user_selected_unlock_report:
                    display_unlock_report("round " + str(Unlock_counter) +
                                          " of going out of piezzo range.")
                ifself.cb_force_unlock.isChecked():
                    if Unlocked_report["round " + str(Unlock_counter) +
                                       " of going out of piezzo range."]["Amount of time spent outside voltage "
                                                                         "range"] > float(self.le_max_unlock_time.text()):
                        num_out_of_voltage_range = 1
                        TimeLastUnlock = 0
                        shut_down()
                        state = STATE_MEASURE
                        UpdateManager.integral_ti = np.zeros(4)
                        ROICam1_Lock.setVisible(False)
                        ROICam2_Lock.setVisible(False)
                        ROICam1_Unlock.setVisible(True)
                        ROICam2_Unlock.setVisible(True)
                        motor_list[0].ch1_v = 75.0
                        motor_list[0].ch2_v = 75.0
                        motor_list[1].ch1_v = 75.0
                        motor_list[1].ch2_v = 75.0
                        successful_lock_time = time.monotonic() - LockTimeStart
                        print("Successfully locked pointing for", successful_lock_time)
                        raise NotImplementedError(
                            'The piezo voltages have gone outside of the allowed range! Stop Lock')
            except InsufficientInformation:
                # catch exception and return to measure state.
                shut_down()
                state = STATE_MEASURE
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
                return
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
            if (motor1_x.size < int(self.le_num_points.text())):
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
            if notself.cb_suppress_piezzo_update.isChecked():
                motor1_x_plot.setData(motor1_x)
                motor1_y_plot.setData(motor1_y)
                motor2_x_plot.setData(motor2_x)
                motor2_y_plot.setData(motor2_y)
            GUI_update_std(UpdateManager.standard_deviation)

        else:
            if cam1_index < 0:
                cam1_x_line.setVisible(False)
                cam1_y_line.setVisible(False)
            if cam2_index < 0:
                cam2_x_line.setVisible(False)
                cam2_y_line.setVisible(False)


    def updateAlign(self):
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
            # ToDO: 2 Make this threshold (the 10.0 below) setable in the GUI.
            if np.any(np.abs(update_voltage - 75.0) > 10.0):
                ROICam1_Lock.setVisible(False)
                ROICam2_Lock.setVisible(False)
                ROICam1_Unlock.setVisible(True)
                ROICam2_Unlock.setVisible(True)
            else:
                ROICam1_Unlock.setVisible(False)
                ROICam2_Unlock.setVisible(False)
                ROICam1_Lock.setVisible(True)
                ROICam2_Lock.setVisible(True)

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


cam1_index = int(self.cb_cam1.currentIndex())
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(UPDATE_TIME)