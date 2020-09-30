from Packages.Cameras import Camera

import random

# Fake Camera that emulates functionality used for testing
class FakeCamera(Camera.Camera):
    kind = "FakeCamera"

    def __init__(self):
        self.frame = None
        self.serial_no = ""


    # Generate random solid color for fake frame
    def update_frame(self):
        pass

    def get_frame(self):
        pass
