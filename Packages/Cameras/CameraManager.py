from typing import List

from Packages.Errors import DeviceNotFoundError
from Packages.Cameras import MightexCameraAPI
# Import all camera types here
from Packages.Cameras import Camera
from Packages.Cameras import FakeCamera
from Packages.Cameras import MightexCamera

# CameraManager()
#   enableMightexAPI()
#   getCameraList()
#   getCamera(index)
#   addCamera(serial_no: str, kind: str)
#   removeCamera(index)
#   removeAllCameras()

class CameraManager():

    def __init__(self):
        # Create resource manager to talk to devices

        self.MightexAPI = None
        

        self.serialList: List[str] = []
        self.cameraList: List[Camera] = []

    def enableMightexAPI(self):
        self.MightexAPI = MightexCameraAPI()
    
    def getCameraList(self):
        return self.cameraList

    def getCamera(self, index):
        return self.cameraList[index]

    # Returns None if port is already in use
    def addCamera(self, serial_no: str, kind: str):
        # ALl cameras in the list must be unique
        if self.serialList.index(serial_no):
            return None
        
        c: Camera = None
        if kind == "Mightex":
            # Try to add camera from serial_no parameter
            for i, ser_no in self.MightexAPI.serial_no:
                if ser_no == serial_no:
                    self.serialList.append(serial_no)
                    c = MightexCamera(self.MightexAPI, serial_no)
                    
            if not c:
                raise DeviceNotFoundError("Invalid serial number!")
            return None
                    
        else:
            c = FakeCamera()

        self.cameraList.append(c)
        return c

    def removeCamera(self, index):
        for i, c in self.cameraList:
            if c.kind == "Mightex":
                c.deactivate_cam()
            self.serialList.remove(i)
            self.cameraList.remove(i)

    def removeAllCameras(self):
        for i in range(len(self.cameraList)):
            self.removeCamera(i)


