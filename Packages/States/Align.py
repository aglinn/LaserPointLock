from Packages.States import State

# Extend QState to implement the align state
# The align state ......

class Align(State.State):

    def preAction(self):
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
        
    def action(self):
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

    def postAction(self):
        pass





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