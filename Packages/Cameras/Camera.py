from abc import ABC, abstractmethod


# Abstract Camera (All other camera typoes inherit from this)

# Camera()
#   update_frame() -> Used if camera needs an external time loop to cause a frame_grab
#   get_frame() -> Grab the most recent frame
#   set_exposure_time(time)
#   set_resolution(res)
#   set_gain(gain)
#   set_decimation(gain)


class Camera(ABC):

    kind = "Camera"

    def __init__(self):
        self.frame = None
        self.serial_no = ""

    @abstractmethod
    def update_frame(self):
        pass

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def set_exposure_time(self, time):
        pass

    @abstractmethod
    def set_resolution(self, res):
        pass

    @abstractmethod
    def set_gain(self, gain):
        pass

    @abstractmethod
    def set_decimation(self, gain):
        pass

    @property
    @abstractmethod
    def exposure_time(self):
        pass

    @property
    @abstractmethod
    def gain(self):
        pass

    @abstractmethod
    def terminate(self):
        pass

