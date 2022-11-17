from Packages.motors import MDT693B_Motor
from Thorlabs_MDT69XB_PythonSDK import MDT_COMMAND_LIB as mdt
import numpy as np

device_list = mdt.mdtListDevices()
print(device_list[0][0], device_list[1][0])

motors = [MDT693B_Motor(device_list[0][0]), MDT693B_Motor(device_list[1][0])]

V = np.array([0])
mdt.mdtGetVoltageAdjustmentResolution(motors[0].handle, V)
mdt.mdtGetLimtVoltage(motors[0].handle, V)
motors[0].ch1_v = 75.000
motors[0].ch2_v = 75.000
motors[1].ch1_v = 75.000
motors[1].ch2_v = 75.000