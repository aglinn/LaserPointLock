from camera import BosonCamera
import timeit
import time
import numpy as np
c = BosonCamera()

print(c.get_averager_state())
c.set_averager_state(1)
c.reboot()
c.close()
time.sleep(5)
c = BosonCamera()
print(c.get_averager_state())
def GetFrame():
    c.update_frame()
    return c.get_frame()

num = 1000
time = timeit.timeit(GetFrame, number=num)
print("Time to capture frame:", time/num)