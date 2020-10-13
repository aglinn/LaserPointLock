from typing import List
from serial.tools import list_ports

from Packages.Errors import DeviceNotFoundError
from Packages.Cameras.MightexCamera_old import MightexCameraAPI
# Import all camera types here
from Packages.Cameras.Camera import Camera
from Packages.Cameras.FakeCamera import FakeCamera
from Packages.Cameras.BosonCamera import BosonCamera
from Packages.Cameras.MightexCamera_old import MightexCamera


# CameraManager()
#   enableMightexAPI()
#   getCameraList()
#   getCamera(index)
#   addCamera(serial_no: str, kind: str)
#   removeCamera(index)
#   removeAllCameras()
cam_list = List[Camera]


class CameraManager():

    # Boson VID and PID:
    VID = 0x09CB
    PID = 0x4007

    def __init__(self):
        # Create resource manager to talk to devices

        self.MightexAPI = None
        
        self._device_list: List[Camera] = [] # All connected devices
        self.selectedDevices: List[int] = [] # Selected devices indecies

        self._active_cam_list = [None, None]

        self.find_devices()

    def enableMightexAPI(self):
        if not self.MightexAPI:
            self.MightexAPI = MightexCameraAPI()

    @property
    def DeviceList(self):
        return self._device_list

    @DeviceList.setter
    def DeviceList(self, device_list:cam_list):
        self._device_list = device_list
        return

    def find_devices(self):

        # Append two fake cameras
        list_of_cameras = []
        list_of_cameras.append(FakeCamera())
        list_of_cameras.append(FakeCamera())

        # Append Mightex
        if self.MightexAPI:
            list_of_cameras.extend(self.MightexAPI.getDeviceList())

        """
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
        """
        self.DeviceList = list_of_cameras
        return
    
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
        for device in self.DeviceList:
            device.terminate()

    @property
    def ActiveCamList(self):
        """
        return list of cameras chosen. Could have 0, 1, or 2 cameras in the appropriate location
        """
        return self._active_cam_list

    @ActiveCamList.setter
    def ActiveCamList(self, CameraList):
        """
        Need to select from available devices based on user input the 2 cameras to use for the Measure, Update, etc. states
        """
        self._active_cam_list = CameraList
        return


    def CaptureCam_1_img(self):
        """
        This function is called by the state manager, and it plays the role of the takeimage function.
        It requires an object, which will be the Update Manager in our case. This Object must have properties: cam_1_img
        and cam_2_img.

        Purpose: Tell cameras to take images and store them as Object.cam_x_img, effectively updating the current images
        stored in the update manager. The update manager will handle the processing of those images for determining update
        voltages and also reporting the images to the GUI.

        This means you should loop over the cameras in your camera list returned by getSelectedCamera and run getframe
        and store the returned frame as Object.cam_x_img.

        Return: Images from camera list
        """
        try:
            self.ActiveCamList[0].update_frame()
            return self.ActiveCamList[0].get_frame()
        except:
            #We could capture attempts to be in the measure state with more than 2 cameras in the list?
            pass

    def CaptureCam_2_img(self):
        try:
            self.ActiveCamList[1].update_frame()
            return self.ActiveCamList[1].get_frame()
        except:
            #We could capture attempts to be in the measure state with more than 2 cameras in the list?
            pass

    def CaptureImages_TwoCameras(self):
        """
        Same as CaptureImages_OneCamera, but assumed 2 cameras are connected.
        """
        self.ActiveCamList[0].update_frame()
        self.ActiveCamList[1].update_frame()
        return [self.ActiveCamList[0].get_frame(), self.ActiveCamList[1].get_frame()]


