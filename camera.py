import usb.core
import usb.util
import numpy as np
import time

# TODO: multithread the frame grabbing

class Camera:
    def __init__(self, dev=None, res=(1280, 1024), exposure_time=50, gain=1, decimation=2):
        r"""
            res = (x_resolution, y_resolution) in pixels
            exposure_time = exposure time of camera in ms
            gain = gain of each pixel (from 1 -> 255)
            decimate = 1 (no decimation) or 2 (decimate by a factor of 2
        """
        if dev is None:
            self.dev = usb.core.find(idVendor=0x04B4, idProduct=0x0228)
            if self.dev is None:
                raise ValueError('Device not found')
            else:
                pass
        else:
            self.dev = dev

        self.dev.set_configuration()
        #r = self.dev.write(0x01, [0x01])
        #r = self.dev.write(0x01, [0x01])
        #r = self.dev.write(0x01, [0x01])
        #r = self.dev.read(0x81, 5)

        self.module_no, self.serial_no, self.date = self.get_device_info()

        self.res = res
        self.exposure_time = exposure_time
        self.gain = gain
        self.decimation = decimation
        self.comm_time = 0
        """Amount of time it takes to pass one frame of data through USB communication.
        Updated when get_frame() is called."""
        self.xres = res[0]
        self.yres = res[1]
        self.set_resolution(res[0], res[1])
        self.set_exposure_time(exposure_time)
        self.set_sensor_Hblanking()
        self.set_main_clock_freq()

    def query(self, command, length, data, rlen):
        """
        Returns a tuple of (cmd, size, return_value)
        """
        self.dev.write(0x01, [command, length, data])
        r = self.dev.read(0x81, rlen + 2)
        return (r[0], r[1], r[2:])

    def set_resolution(self, xres, yres):
        _xres = xres.to_bytes(2, byteorder='big')
        _yres = yres.to_bytes(2, byteorder='big')
        decimation = 0 if self.decimation == 1 else 1
        result = self.dev.write(0x01, [0x60, 7, _xres[0], _xres[1], _yres[0], _yres[1], decimation])
        self.res = (xres, yres)
        return result

    def get_device_info(self):
        _, _, info = self.query(0x21, 1, 0x00, 43)
        info = info.tobytes().decode('utf-8')
        # print("[DEVICE]")
        # print("ModuleNo: %s\nSerialNo: %s\nManufacturingDate: %s\n"
        #     % (info[1:15], info[15:29], info[29:43]))
        _, _, firmware = self.query(0x01, 1, 0x01, 3)
        # print("[FIRMWARE]")
        # print("%s.%s.%s\n"
        #     % (firmware[0], firmware[1], firmware[2]))
        return info[1:15], info[15:29], info[29:43]

    def get_frame_prop(self):
        _, _, frameProp = self.query(0x33, 1, 0, 18)
        frameProp = frameProp.tobytes()
        rowSize = int.from_bytes(frameProp[0:2], byteorder='big')
        colSize = int.from_bytes(frameProp[2:4], byteorder='big')
        bin = frameProp[4]
        exposure_time = int.from_bytes(frameProp[5:7], byteorder='big')
        r_gain = frameProp[7]
        g_gain = frameProp[8]
        b_gain = frameProp[9]
        x_start = int.from_bytes(frameProp[10:12], byteorder='big')
        y_start = int.from_bytes(frameProp[12:14], byteorder='big')
        frame_invalid = frameProp[14]
        time_stamp = int.from_bytes(frameProp[16:18], byteorder='big')
        return rowSize, colSize, bin, exposure_time, r_gain, g_gain, b_gain, x_start, y_start, time_stamp

    def set_main_clock_freq(self):
        # Set normal clock speed
        self.dev.write(0x01, [0x32, 1, 1])

    def set_sensor_Hblanking(self):
        # Set longest Hblanking
        self.dev.write(0x01, [0x36, 1, 2])

    def set_exposure_time(self, time):
        # time in ms
        time_mult = max(1, min(15000, int(time/0.05)))
        time_mult = time_mult.to_bytes(length=2, byteorder='big')
        self.dev.write(0x01, [0x63, 2, time_mult[0], time_mult[1]])
        self.exposure_time = max(0.05, min(750, time))
    
    def set_gain(self, gain):
        if getattr(gain,'__iter__',False):
            # gain is iterable, so treat as a list/tuple
            if not len(gain) == 3:
                raise ValueError("Gain tuple must consist of exactly three values")
            gain = [max(0, min(64, int(round(x*8)))) for x in gain]
            self.dev.write(0x01, [0x62, 3, gain[0], gain[1], gain[2]])
            self.gain = gain
        else:
            # single value passed in
            gain = max(0, min(64, int(round(gain*8))))
            self.dev.write(0x01, [0x62, 3, gain, gain, gain])
            self.gain = gain # apply after setting succesfully applied

    def set_decimation(self, value):
        if isinstance(value, bool):
            self.decimation = 2 if value else 1
            self.set_resolution(self.xres, self.yres)
        else:
            raise ValueError("Decimation must be a boolean")

    def pixel_to_um(self, pixels):
        """
        Convert from pixels to um assuming SCE-B013-U camera
        """
        return pixels*5.5*self.decimation

    def get_frame(self):
        x_pixels = int(self.res[0]//self.decimation)
        y_pixels = int(self.res[1]//self.decimation)
        buff_length = int(x_pixels*y_pixels/2/512)
        array1 = np.empty((buff_length, 512), dtype='B')
        array2 = np.empty((buff_length, 512), dtype='B')

        # Get Camera trigger state and check resolution
        cmd, size, trigState = self.query(0x35, 1, 0, 6)
        trigger = int(trigState[0])
        rowSize = int.from_bytes(trigState[1:3], byteorder='big')
        colSize = int.from_bytes(trigState[3:5], byteorder='big')
        bin = int(trigState[5])
        if rowSize != self.res[0]:
            print("xres  = %d, but self.res[0] = %d" % (rowSize, self.res[0]))
            # raise IOError('Resolution not configured!')
        if colSize != self.res[1]:
            print("yres  = %d, but self.res[1] = %d" % (colSize, self.res[1]))
            # raise IOError('Resolution not configured!')

        # Get image data - begin frame acquisition
        self.dev.write(0x01, [0x34, 1, 1])
        # Now get the data
        start_time = time.time()
        for x in range(0, buff_length):
            array1[x] = self.dev.read(0x82, 512)
            array2[x] = self.dev.read(0x86, 512)
        # Measure the amount of time it took to transfer information
        self.comm_time = time.time() - start_time - self.exposure_time/1000
        array1 = array1.flatten().reshape(y_pixels//2, x_pixels)
        array2 = array2.flatten().reshape(y_pixels//2, x_pixels)

        # Get current frame property
        self.get_frame_prop()

        # Construct the frame
        image = np.empty((y_pixels, x_pixels), dtype='B')
        image[::2, :] = array1
        image[1::2, :] = array2
        return image

import random
class FakeCamera:
    def __init__(self, **kwargs):
        """
        module_no
        serial_no
        date
        res
        exposure_time
        gain
        decimation
        """
        self.module_no = 'SCE-B013-U'
        self.serial_no = kwargs.get('serial_no', '{0:02}-{1:06}'.format(random.randint(10, 99), random.randint(0, 999999)))
        self.date = '011219'

        self.res = kwargs.get('res', (1024, 1280))
        self.exposure_time = kwargs.get('exposure_time', 0.05)
        self.gain = kwargs.get('gain', 1)
        self.decimation = kwargs.get('decimation', 1)
        self.xcen = kwargs.get('xcen', 0)
        self.ycen = kwargs.get('ycen', 0)
        self.comm_time = 0
        """Amount of time it takes to pass one frame of data through USB communication.
        Updated when get_frame() is called."""
        self.xres = self.res[0]
        self.yres = self.res[1]
        self.set_resolution(self.res[0], self.res[1])
        self.set_exposure_time(self.exposure_time)
        self._grid = np.meshgrid((np.arange(self.xres) - self.xres/2)//self.decimation,
        (np.arange(self.yres) - self.yres/2)//self.decimation, indexing='ij')
    
    def set_resolution(self, xres, yres):
        self.xres = xres
        self.yres = yres
        self._grid = np.meshgrid((np.arange(self.xres) - self.xres/2),
        (np.arange(self.yres) - self.yres/2), indexing='ij')
    
    def set_exposure_time(self, time):
        self.exposure_time = time
    
    def set_gain(self, gain):
        self.gain = gain*8
    
    def set_decimation(self, value):
        if isinstance(value, bool):
            self.decimation = 2 if value else 1
            self.set_resolution(self.xres, self.yres)
        else:
            raise ValueError("Decimation must be a boolean")
    
    def pixel_to_um(self, pixels):
        return pixels*5.5*self.decimation
    
    def get_frame(self):
        # Generate a new center
        self.xcen = min(self.xres//2, max(-self.xres//2, self.xcen + random.randint(-10, 10)))
        self.ycen = min(self.yres//2, max(-self.yres//2, self.ycen + random.randint(-10, 10)))
        # print('I am cam', self.serial_no, 'my xcen', self.xcen + self.xres//2, 'my ycen', self.ycen + self.yres//2)
        # Simulate a laser spot
        width = 0.02*self._grid[0].shape[0]
        image = np.round(245*np.exp(-((self._grid[0] - self.xcen)**2 + (self._grid[1] - self.ycen)**2)/(2*width**2))).astype(np.uint8)
        return image