"""
File:                       ULTI02.py

Library Call Demonstrated:  mcculw.ul.t_in_scan()

Purpose:                    Scans temperature input channels.

Demonstration:              Displays the temperature inputs on a
                            range of channels.

Special Requirements:       Unless the board at BoardNum(=0) does not use
                            EXP boards for temperature measurements(the
                            CIO-DAS-TC or USB-2001-TC for example), it must
                            have an A/D converter with an attached EXP
                            board.  Thermocouples must be wired to EXP
                            channels selected.
"""
from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport
import time
import numpy as np
from mcculw import ul
from mcculw.enums import TempScale, ErrorCode
import matplotlib.pyplot as plt
from mcculw.enums import InterfaceType
from mcculw.ul import ULError
from mcculw.device_info import DaqDeviceInfo
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QTimer


class TemperatureLogger:
    def __init__(self, board_num=0, use_device_detection=False, temp_scale='C'):
        # By default, the system relies on Instacal configured devices and connects to the one specified by board num
        # If use_device_detection is set to False, the board_num property needs
        # to match the desired board number configured with Instacal.
        # Instead, can provide the board number and use_device_detection=true to connect to the board number-th device
        print("Init temperature Logger")
        if 'c' in temp_scale or 'C' in temp_scale:
            self.temp_scale = TempScale.CELSIUS
        elif 'f' in temp_scale or 'F' in temp_scale:
            self.temp_scale = TempScale.FAHRENHEIT
        elif 'k' in temp_scale or 'K' in temp_scale:
            self.temp_scale = TempScale.KELVIN
        else:
            self.temp_scale = None
            raise Exception("Must set temp scale to farenheight, kelvin, or celsius with first letter of temp_scale"
                            " mattering.")
        self.board_num = board_num

        self.running = False

        if use_device_detection:
            self.configure_device()

        self.device_info = DaqDeviceInfo(self.board_num)
        self.ai_info = self.device_info.get_ai_info()
        if not self.ai_info.temp_supported:
            print("Whoops need a temperature logging device that is temp_supported, e.g. OM-NET-TC or OM-USB-TC")

        # get the channels to read
        self.scan = False
        self.low_chan = []
        self.high_chan = []
        ret = self.get_scan_range()
        if not ret:
            print("Whoops temperature logging device has no channels with a Thermocouple properly connected.")
        return

    def get_scan_range(self):
        """
        Figure out the range of channels to scan and set self.low_channels and self.high_channels as a list of starting
        and stopping values of channels that are connected between start and stop value. Set self.scan to false if only
        one value to read.
        Returns
        -------
        Bool True if there are channels with Tc to read otherwise False.
        """
        connected_channels = []
        for channel in range(10):
            try:
                value = ul.t_in(self.board_num, channel, self.temp_scale)
                connected_channels.append(channel)
            except ULError:
                pass
        if not connected_channels:
            return False
        elif len(connected_channels) == 1:
            self.low_chan = connected_channels
            self.high_chan = connected_channels
            self.scan = False
            return True
        self.low_chan = [connected_channels[0]]
        self.high_chan = []
        last_channel = connected_channels[0]
        for channel in connected_channels[1:]:
            if channel - 1 == last_channel:
                last_channel = channel
            else:
                self.high_chan.append(last_channel)
                self.low_chan.append(channel)
                last_channel = channel
        self.high_chan.append(channel)
        return True

    def update_values(self):
        for i in range(len(self.low_chan)):
            l = self.low_chan[i]
            h = self.high_chan[i]
            data_array = np.array([])
            try:
                # Get the values from the device (optional parameters omitted)
                err_code, data = ul.t_in_scan(self.board_num, l, h, self.temp_scale)

                # Check err_code for OUTOFRANGE or OPENCONNECTION. All other
                # error codes will raise a ULError and are checked by the except
                # clause.
                if err_code == ErrorCode.OUTOFRANGE:
                    print(
                        "A thermocouple input is out of range.")
                elif err_code == ErrorCode.OPENCONNECTION:
                    print(
                        "A thermocouple input has an open connection.")
                data_array = np.concatenate((data_array, np.asarray(data)), axis=0)
            except ULError as e:
                self.stop()
                self.show_ul_error(e)
            return data_array

    def configure_device(self):
        ul.ignore_instacal()
        devices = ul.get_daq_device_inventory(InterfaceType.ANY)
        if not devices:
            raise ULError(ErrorCode.BADBOARD)

        # Add the first DAQ device to the UL with the specified board number
        ul.create_daq_device(self.board_num, devices[0])

    @staticmethod
    def show_ul_error(ul_error):
        message = 'A UL Error occurred.\n\n' + str(ul_error)
        print(message)
        return


class QTemperatureLogger(TemperatureLogger, QObject):
    """data_return = pyqtSignal(np.ndarray)"""
    finished_logging = pyqtSignal()

    def __init__(self, board_num=0, use_device_detection=False, temp_scale='C', sampling_frequency: float = 10):
        """
        sampling frequency is in Hz
        """
        # init temperature logger
        super(QTemperatureLogger, self).__init__(board_num, use_device_detection, temp_scale)
        # init QObject
        super(TemperatureLogger, self).__init__()
        # Create timer to run the logger
        self.timer = QTimer(self)
        interval = int(1000/sampling_frequency)  # ms
        self.timer.setInterval(interval)
        self.timer.setSingleShot(False)
        self.timer.timeout.connect(self.log_data)
        self.data = None
        self.file_path = None
        return

    @pyqtSlot()
    def log_data(self):
        data = self.update_values()
        data = data.reshape(1, data.size)
        self.data = np.concatenate((self.data, data), axis=0)
        return

    @pyqtSlot()
    def start(self, save_directory):
        data = self.update_values()
        data = data.reshape(1, data.size)
        self.data = data
        self.timer.start()
        self.file_path = save_directory + '/temperature'
        return

    @pyqtSlot()
    def stop(self):
        """
        Stop logging data.
        """
        self.timer.stop()
        # self.data_return.emit(self.data)
        self.store_data()
        self.data = None
        self.file_path = None
        self.finished_logging.emit()
        return

    def store_data(self):
        """
        Write the data to file.
        """
        temperature_data = np.memmap(self.file_path, dtype='float64', mode='w+',
                                     shape=self.data.shape, order='C')
        temperature_data[:] = self.data[:]
        temperature_data.flush()
        return

def test_logger():
    # Start the example
    temp_log = TemperatureLogger()
    data_scan = temp_log.update_values()
    data_scan = data_scan.reshape(4, 1)
    for i in range(100):
        time.sleep(0.1)
        data = temp_log.update_values()
        data = data.reshape(4, 1)
        data_scan = np.concatenate((data_scan, data), axis=1)
    print(data_scan.shape)
    fig, ax = plt.subplots(dpi=200)
    ax.plot(data_scan[0, :])
    ax.plot(data_scan[1, :])
    ax.plot(data_scan[2, :])
    ax.plot(data_scan[3, :])
    plt.show()
    return

# Start the example if this module is being run
if __name__ == "__main__":
    test_logger()