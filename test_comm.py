import usb.core
import usb.util
import numpy as np
import time

class Camera:
    def __init__(self, res=(1280, 1024), exposure_time=200, gain=8, decimation=1):
        r"""
            res = (x_resolution, y_resolution) in pixels
            exposure_time = exposure time of camera in ms
            gain = gain of each pixel (from 1 -> 255)
            decimate = 1 (no decimation) or 2 (decimate by a factor of 2)
        """
        self.dev = usb.core.find(idVendor=0x04B4, idProduct=0x0228)
        if self.dev is None:
            raise ValueError('Device not found')
        else:
            pass

        self.dev.set_configuration()
        #r = self.dev.write(0x01, [0x01])
        #r = self.dev.write(0x01, [0x01])
        #r = self.dev.write(0x01, [0x01])
        #r = self.dev.read(0x81, 5)

        self.get_device_info()

        self.res = res
        self.exposure_time = exposure_time
        self.gain = gain
        self.decimation = decimation
        self.comm_time = 0
        self.set_resolution(res[0], res[1])
        self.set_exposure_time(exposure_time)
        self.set_sensor_Hblanking()
        self.set_main_clock_freq()
        time.sleep(2)

    def query(self, command, length, data, rlen):
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
        cmd, size, info = self.query(0x21, 1, 0x00, 43)
        info = info.tobytes().decode('utf-8')
        print("[DEVICE]")
        print("ModuleNo: %s\nSerialNo: %s\nManufacturingDate: %s\n"
            % (info[1:15], info[15:29], info[29:43]))
        cmd, size, firmware = self.query(0x01, 1, 0x01, 3)
        print("[FIRMWARE]")
        print("%s.%s.%s\n"
            % (firmware[0], firmware[1], firmware[2]))

    def get_frame_prop(self):
        cmd, size, frameProp = self.query(0x33, 1, 0, 18)
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
        time_mult = int(time/0.05)
        time_mult = time_mult.to_bytes(length=2, byteorder='big')
        self.dev.write(0x01, [0x63, 2, time_mult[0], time_mult[1]])
        self.exposure_time = time

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
        self.comm_time = time.time() - start_time
        print('Comm time:', self.comm_time)
        array1 = array1.flatten().reshape(y_pixels//2, x_pixels)
        array2 = array2.flatten().reshape(y_pixels//2, x_pixels)

        # Get current frame property
        self.get_frame_prop()

        # Construct the frame
        image = np.empty((y_pixels, x_pixels), dtype='B')
        image[::2, :] = array1
        image[1::2, :] = array2
        return image


if __name__ == '__main__':
    c = Camera(decimation=2)
    img = c.get_frame()

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import time
    fig, ax = plt.subplots()
    vert_line = ax.axvline(img.shape[1]//2)
    horz_line = ax.axhline(img.shape[0]//2)
    vert_line_com = ax.axvline(img.shape[1]//2, color='r')
    horz_line_com = ax.axhline(img.shape[0]//2, color='r')
    img_display = ax.imshow(img)
    X, Y = np.meshgrid(np.arange(img.shape[0]), np.arange(img.shape[1]))
    X = X.T
    Y = Y.T

    def find_com(img, X, Y):
        sum_x = np.sum(np.sum(img*X))
        sum_y = np.sum(np.sum(img*Y))
        return sum_y/np.sum(np.sum(img)), sum_x/np.sum(np.sum(img))

    def update(frame):
        start_time = time.time()
        img = c.get_frame()
        print("Max pixel value = %d" % (np.max(np.max(img))))
        img_display.set_data(img)
        x, y = find_com(img, X, Y)
        print("Horiz Cursor: %.1f\tVert Cursor: %.1f" % (y, x))
        vert_line_com.set_xdata([x, x])
        horz_line_com.set_ydata([y, y])
        print("Time for frame update: %.3f (ms)" % (time.time() - start_time))
        return img_display, horz_line, vert_line, horz_line_com, vert_line_com
    ani = FuncAnimation(fig, update, frames=1, interval=250, blit=True)

    plt.show()
