from Packages.Cameras import Camera

from Packages.Errors import DeviceNotFoundError
from Packages.Cameras.MightexCameraAPI import MightexCameraAPI


# Mightex Camera class
class MightexCamera(Camera.Camera):

    kind = "Mightex"

    def __init__(self, engine: MightexCameraAPI, serial_no: str):
        super.__init__()
        
        self.engine: MightexEngine = engine
        if serial_no not in self.engine.serial_no:
            raise DeviceNotFoundError("Serial number {} is not connected to computer".format(serial_no))
        self.serial_no = serial_no
        self.frame = None
        self._exposure_time = 0.05
        # self.run_thread = CameraThread(self)
        # self.signals = CameraSignals()

    def get_frame(self, serial_no, n=1, getExposureTime=0, getTimeStamp=0):
        n = max(n, 1)  # if n < 1, set n = 1
        if serial_no not in self.active_devs:
            return None
        else:
            return self.frame

    def update_frame(self):
        #lock.lockForRead()
        # print("serial number just before calling the engine get frame function:",self.serial_no)
        self.frame, self.time = self.engine.get_frame(self.serial_no, 1, getTimeStamp=1)
        #lock.unlock()

    def set_exposure_time(self, time):
        self.engine._setExposure(self.serial_no, time)
        self.frame = self.engine.get_frame(self.serial_no, 1, getExposureTime=1)
        self._exposure_time = self.engine._getExposure(self.serial_no)

    def set_resolution(self, res):
        self.engine._setResolution(self.engine.dev_num[self.serial_no], res[0], res[1], 0)

    def set_gain(self, gain):
        pass

    def set_decimation(self, gain):
        pass

    @property
    def exposure_time(self):
        return self._exposure_time

    @property
    def gain(self):
        return 1

    @property
    def dev_num(self):
        return self.engine.dev_num[self.serial_no]            

    def _setExposure(self, serial_no, exposureTime):
        # C code requires int in units of 50 us. 1 = 50 us. 10 = 500 us, etc; so convert ms to this unit.
        exposureTimeInt = int(exposureTime*1000/50)
        self._setExposureTime(ctypes.c_int(self.dev_num[serial_no]), ctypes.c_int(exposureTimeInt))

    def _getExposure(self,serial_no):
        return self.exposureTime[serial_no]

    def _frameFromFramePtr(self, framePtr):
        # Get resolution from mightex engine
        res = self.resolution[serial_no]
        size = reduce(operator.mul, res, 1) + 56
        frame_type = ctypes.c_ubyte*size
        frame = frame_type()

        #lock.lockForWrite()
        print("serial number:",serial_no,"dev number:", self.dev_num[serial_no])
        # print("dev num, just prior to get frame:",self.dev_num[serial_no])
        self.BufferPoint[serial_no] = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
        """
        # print("Device Number:",self.dev_num[serial_no], "SerialNumber:", serial_no,"Pointer:", self.BufferPoint[serial_no])
        # if self.BufferPoint[self.serial_no[0]] == self.BufferPoint[self.serial_no[1]]:
            # print("Both camera's buffer pointer point to the same space in memory.")
        #lock.unlock()
        #lock.lockForRead()
        # print("dev num, just prior to pymemoryview:",self.dev_num[serial_no])
        """
        self.buffer[serial_no] = ctypes.pythonapi.PyMemoryView_FromMemory(self.BufferPoint[serial_no], size)
        # print("dev num, just prior to copying from buffer:", self.dev_num[serial_no])
        self.z[serial_no] = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
        if (getTimeStamp+getExposureTime)>0:
            FrameInfo = np.frombuffer(self.buffer[serial_no], dtype=np.uint8, count=size)[24:44].copy()
            if getExposureTime:
                self.exposureTime[serial_no] = int.from_bytes(FrameInfo[0:4], byteorder='little')*50.0/1000.0
            TimeStamp = int.from_bytes(FrameInfo[16:20], byteorder='little')
        #lock.unlock()
        for i in range(n - 1):
            #lock.lockForRead()
            self.BufferPoint[serial_no] = self._getFrame(ctypes.c_int(0), ctypes.c_int(self.dev_num[serial_no]), frame)
            self.buffer[serial_no] = ctypes.pythonapi.PyMemoryView_FromMemory(self.BufferPoint[serial_no], size)
            self.z[serial_no] += np.frombuffer(self.buffer, dtype=np.uint8, count=size)[56:].copy().reshape(res[::-1])
            #lock.unlock()
        if n > 1:
            self.z[serial_no] = self.z[serial_no].astype(np.float) / n
        self.z[serial_no] = self.z[serial_no].astype(np.float)
        # ("dev num just prior to returning the frame", self.dev_num[serial_no])
        if getTimeStamp:
            return self.z[serial_no], TimeStamp
        else:
            return self.z[serial_no]

