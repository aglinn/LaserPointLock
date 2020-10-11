from typing import List
from serial.tools import list_ports

from Packages.Errors import DeviceNotFoundError
from Packages.Cameras import MightexCameraAPI
# Import all camera types here
from Packages.Cameras.Camera import Camera
from Packages.Cameras.FakeCamera import FakeCamera
from Packages.Cameras.MightexCamera import MightexCamera
from Packages.Cameras.BosonCamera import BosonCamera


# CameraManager()
#   enableMightexAPI()
#   getCameraList()
#   getCamera(index)
#   addCamera(serial_no: str, kind: str)
#   removeCamera(index)
#   removeAllCameras()

class CameraManager():

    # Boson VID and PID:
    VID = 0x09CB
    PID = 0x4007

    def __init__(self):
        # Create resource manager to talk to devices

        self.MightexAPI = None
        
        self.deviceList: List[Camera] = [] # All connected devices
        self.selectedDevices: List[int] = [] # Selected devices indecies

        self.getDeviceList()

    def enableMightexAPI(self):
        if not self.MightexAPI:
            self.MightexAPI = MightexCameraAPI()

    def getDeviceList(self):
        if len(self.deviceList) > 0:
            return self.deviceList

        # Append two fake cameras
        self.deviceList = []
        self.deviceList.append(FakeCamera(1))
        self.deviceList.append(FakeCamera(2))

        # Append Mightex
        if self.MightexAPI:
            self.deviceList.append(self.MightexAPI.getDeviceList())

        # Append Bosons
        #TODO: Figure out how to correctly connect two BOSONs.
        device_id = None
        for device in list_ports.comports():
                if device.vid == VID and device.pid == PID:
                    port = device.device
                    # self.deviceList.append(port)
                    c = BosonCamera(port=port, device_id=device_id)
                    print(port)
                    ffc_state = 0
                    c.do_ffc()
                    while ffc_state == 0:
                        ffc_state = c.get_ffc_state()
                    self.deviceList.append(c)
                    device_id = 1

        return self.deviceList
    
    # Returns None if port is already in use
    def selectCamera(self, index: int):
        # All cameras in the list must be unique
        if not self.selectedDevices.index(index):
            self.selectedDevices.append(index)
            return index
        else:
            return -1
    
    # Returns camera object given index of device index stored in selected devices
    def getSelectedCamera(self, index: int):
        if index >=0 and index < len(self.selectedDevices):
            return self.deviceList[index]
        else:
            return None


    def terminate(self):
        # Terminate all cameras
        for device in self.deviceList:
            device.terminate()

    def CaptureImages(self, Object):
        """
        This function is called by the state manager, and it plays the role of the takeimage function.
        It requires an object, which will be the Update Manager in our case. This Object must have properties: cam_1_img
        and cam_2_img.

        Purpose: Tell cameras to take images and store them as Object.cam_x_img, effectively updating the current images
        stored in the update manager. The update manager will handle the processing of those images for determining update
        voltages and also reporting the images to the GUI.

        This means you should loop over the cameras in your camera list returned by getSelectedCamera and run getframe
        and store the returned frame as Object.cam_x_img.

        Return: Nothing from this function.
        """
        pass


