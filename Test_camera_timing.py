from Packages.camera import MightexCamera
from Packages.camera import MightexEngine
import numpy as np
import time


mightex_engine = MightexEngine([1, 1])
cam = MightexCamera(mightex_engine, mightex_engine.serial_no[1])

times = np.arange(5)
cam.update_frame()
cam_start_time = cam.time
start_time = time.monotonic()
# print("Start time is", time.monotonic())
print(cam.serial_no)
cam.engine.update_working_mode()

for t in times:
    print("sleep for ", t)
    time.sleep(t)
    cam.update_frame()
    print("update time delta is", time.monotonic() - start_time)
    start_time = time.monotonic()
    print("camera update time delta is ", cam.time-cam_start_time)
    cam_start_time = cam.time
