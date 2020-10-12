from Packages.States import State

# Extend State to implement the locked state
# The locked state ......

class Locked(State.State):

    label = "LOCKED"

    def __init__(self):
        super().__init__()

        lockTimeStart = 0

    def preAction(self):
        pass
        
    def action(self):

        #Tell cameras to report new frames to update manager
        UpdateManager.cam_1_img, UpdateManager.cam_2_img = CameraManager.CaptureImages_TwoCameras()

        # Tell the update manager the current piezo voltages
        #UpdateManager.V0 = np.array([motor_list[0].ch1_v, motor_list[0].ch2_v, motor_list[1].ch1_v,
                                       #motor_list[1].ch2_v])
        #probably want to pass this information at the end of a loop rather than the front of the logic loop; so this
        #information is available for the update GUI function.
        UpdateManager.V0 = MotorManager.ReportVoltages()

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
                MotorManager.ResetMotors()
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
                # Apply Voltages
                MotorManager.ApplyVoltages(update_voltage)

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

    def postAction(self):
        pass

