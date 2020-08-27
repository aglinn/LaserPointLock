import numpy as np
import nfft


class InsufficientInformation(Exception):

    def __init__(self, string):
        Exception.message(string)


class UpdateManager:

    def __init__(self):
        self.calibration_matrix = None
        self.set_pos = None
        self.dV = None
        self.update_voltage = None
        self.cam_1_com = None
        self.cam_2_com = None
        self.cam_1_img = None
        self.cam_2_img = None
        self._dx = []
        self.V0 = None
        self._t1 = []
        self._t2 = []
        self._t1_wrapper_count = 0
        self._t2_wrapper_count = 0

    def get_update(self):
        """
        This code finds the next voltages to apply to the piezo.
        """
        if self.calibration_matrix is None or self.set_pos is None:
            raise InsufficientInformation("You must first provide a calibration matrix and a set position.")

        # Calculate dX
        # The convention here is how much has the position on the camera changed from the set point.
        self.dx = self.com()-self.set_pos

        # Because of our convention for dX, the meaning of dV is how much would the voltages have changed to result
        # in the change observed in dX
        dV = np.matmul(self.calibration_matrix, self.dx[-1, :])
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.
        # voltage to be set on the motors
        # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
        # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
        # so we subtract that dV restoring us to the old position.
        self.update_voltage = self.V0 - self.dV
        return self.update_voltage

    def calc_dx(self):
        self.dx = self.com()-self.set_pos
        return

    def store_data(self, state):
        """
        STATE_MEASURE = 0
        STATE_CALIBRATE = 1
        STATE_LOCKED = 2
        STATE_ALIGN = 3
        """
        if state == 0:
            state = "Measure"
        elif state == 1:
            state = "Calibrate"
        elif state == 2:
            state = "Locked"
        elif state == 3:
            state = "Align"
        date = str(np.datetime64('today', 'D'))
        if len(self.dx) > 0:
            filename = 'Data/' + date + state + '_dx'
            np.savetxt(filename, self.dx, fmt='%f')
        if len(self.t1) > 0:
            filename = 'Data/' + date + state + '_t1'
            np.savetxt(filename, self.t1, fmt='%f')
        if len(self.t2) > 0:
            filename = 'Data/' + date + state + '_t2'
            np.savetxt(filename, self.t2, fmt='%f')
        return

    def reset_data(self):
        self._dx = []
        self._t1 = []
        self._t2 = []
        self._t1_wrapper_count = 0
        self._t2_wrapper_count = 0
        return

    @property
    def standard_deviation(self):
        return np.std(self.dx, axis=1)

    @property
    def update_voltage(self):
        return self._update_voltage

    @property
    def frequency_domain(self):
        pass

    @property
    def frequency_data(self):
        dt1_min = np.amin(self.t1[1:-1]-self.t1[0:-2])
        dt2_min = np.amin(self.t2[1:-1]-self.t2[0:-2])
        T1 = self.t1[-1]-self.t1[0]
        T2 = self.t2[-1]-self.t2[0]
        N1 = np.floor(T1/dt1_min)
        if N1 % 2 == 1:
            N1 = N1-1
        N2 = np.floor(T2/dt2_min)
        if N2 % 2 == 1:
            N2 = N2-1
        FT_cam1_dx = nfft.nfft_adjoint(self.t1, self.dx[:, 0], N1, m)
        nfft.nfft_adjoint()
        return

    @update_voltage.setter
    def update_voltage(self, vector):
        self._update_voltage = vector
        return

    @property
    def dV(self):
        return self._dV

    @dV.setter
    def dV(self, vector):
        self._dV = vector
        return

    @property
    def cam_1_com(self):
        return self._cam_1_com

    @cam_1_com.setter
    def cam_1_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 1
        """
        self._cam_1_com = vector
        return

    @property
    def cam_2_com(self):
        return self._cam_2_com

    @cam_2_com.setter
    def cam_2_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 2
        """
        self._cam_2_com = vector
        return

    def com(self):
        return np.concatenate((self.cam_1_com, self.cam_2_com), axis=0)

    @property
    def dx(self):
        """
        Returns dx as a matrix, where row number corresponds to t, and columns to positions as defined in the
        setter method below.
        """
        return np.asarray(self._dx)

    @dx.setter
    def dx(self, vector):
        """
        vector defines the current displacement vector.
        vector has 4 float elements and is a row vector. This is a displacement vector along the dimensions listed below
        The convention for the vector will vector = (camera 1 row position, camera 1 column position,
        camera 2 row position, camera 2 column position).
        """
        self._dx.append(vector)
        return

    @property
    def V0(self):
        return self._V0

    @V0.setter
    def V0(self, vector):
        self._V0 = vector
        return

    @property
    def calibration_matrix(self):
        return self._calibration_matrix

    @calibration_matrix.setter
    def calibration_matrix(self, matrix):
        self._calibration_matrix = matrix
        return

    @property
    def set_pos(self):
        return self._set_pos

    @set_pos.setter
    def set_pos(self, vector):
        self._set_pos = vector
        return

    @property
    def cam_1_img(self):
        return self._cam_1_img

    @cam_1_img.setter
    def cam_1_img(self, img):
        self._cam_1_img = img
        return

    @property
    def cam_2_img(self):
        return self._cam_2_img

    @cam_2_img.setter
    def cam_2_img(self, img):
        self._cam_2_img = img
        return

    @property
    def t1(self, index=None):
        """Camera 1 image timestamp"""
        return np.asarray(self._t1)

    @t1.setter
    def t1(self, value):
        if len(self.t1 > 0):
            if value < self.t1[-1]:
                self._t1_wrapper_count += 1  # Convert to a monotonic timestamp
        self._t1.append(self._t1_wrapper_count*(65535+1)+value)
        return

    @property
    def t2(self):
        """Camera 2 image timestamp"""
        return np.asarray(self._t2)

    @t2.setter
    def t2(self, value):
        if len(self.t2 > 0):
            if value < self.t2[-1]:
                self._t2_wrapper_count += 1  # Convert to a monotonic timestamp
        self._t2.append(self._t2_wrapper_count * (65535 + 1) + value)
        return


class PIDUpdateManager(UpdateManager):

    def __init__(self):
        super().__init__()
        self._dt = None
        self._integral_ti = None
        self._P = None
        self._TI = None
        self._TD = None

    def get_update(self):
        """
        This code finds the next voltages to apply to the piezo.
        """
        if self.calibration_matrix is None or self.set_pos is None:
            raise InsufficientInformation("You must first provide a calibration matrix and a set position.")

        # Calculate dX
        # The convention here is how much has the position on the camera changed from the set point.
        self.dx = self.com()-self.set_pos
        if len(self.t1) > 1:  # PID only makes sense if there are at least 2 data points. If only one, just do P.
            # Calculate dt
            self.calc_dt()
            derivative = self.calc_derivative()
            self.calc_integral()

            # Find dX weighted total from PID.
            dx = self.P*(self.dx[-1, :] + self.integral_ti/self.TI + self.TD*derivative)
        else:
            dx = self.dx[-1, :]
        # Because of our convention for dX, the meaning of dV is how much would the voltages have changed to result
        # in the change observed in dX
        dV = np.matmul(self.calibration_matrix, dx)
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.
        # voltage to be set on the motors
        # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
        # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
        # so we subtract that dV restoring us to the old position.
        self.update_voltage = self.V0 - self.dV
        return self.update_voltage

    def calc_integral(self):
        increment_cam1 = self.dt[0]*self.dx[-1, 0:2]
        increment_cam2 = self.dt[1]*self.dx[-1, 2:4]
        self._integral_ti += np.concatenate([increment_cam1, increment_cam2], axis=0)  # Currently using the most
        # simplistic numerical integral, may want to improve
        return

    def calc_derivative(self):
        derivative_cam1 = (self.dx[-1, 0:2] - self.dx[-2, 0:2]) / self.dt[0]
        derivative_cam2 = (self.dx[-1, 2:4] - self.dx[-2, 2:4]) / self.dt[1]
        return np.concatenate([derivative_cam1, derivative_cam2], axis=0)

    def calc_dt(self):
        dt1 = self.t1[-1] - self.t1[-2]
        dt2 = self.t2[-1] - self.t2[-2]
        self.dt = np.array([dt1, dt2])
        return

    @property
    def integral_ti(self):
        return self._integral_ti

    @integral_ti.setter
    def integral_ti(self, vector):
        self._integral_ti = vector
        return

    @property
    def dt(self):
        return self._dt

    @dt.setter
    def dt(self, vector):
        self._dt = vector
        return

    @property
    def P(self):
        return self._P

    @P.setter
    def P(self, value):
        self._P = value
        return

    @property
    def TI(self):
        return self._TI

    @TI.setter
    def TI(self, value):
        self._TI = value
        return

    @property
    def TD(self):
        return self._TD

    @TD.setter
    def TD(self, value):
        self._TD = value
        return