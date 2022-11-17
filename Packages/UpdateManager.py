import numpy as np
import nfft
import time
from lmfit import Parameters, minimize

class update_out_of_bounds(Exception):

    pass

class InsufficientInformation(Exception):

    pass


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
        self._frequency_domain = None

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
        update_voltage = self.V0 - self.dV
        if np.any(update_voltage > 150) or np.any(update_voltage < 0):
            exception_message = ''
            for i in range(4):
                if update_voltage[i] < 0:
                    exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) +' '
                if update_voltage[i] > 150:
                    exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) +' '
            raise update_out_of_bounds(exception_message)
        else:
            self.update_voltage = update_voltage
            return self.update_voltage

    def fit_update(self):
        """
        When the update voltages are out of their allowed range, I want to try to find an update that minimizes
        my dx in the future step, constrained by the allowed piezo ranges. That is what this function does.
        """
        P = Parameters()
        P.add('dv_0', 0, min = -self.V0[0], max = 150.0 - self.V0[0])
        P.add('dv_1', 0, min=-self.V0[1], max=150.0 - self.V0[1])
        P.add('dv_2', 0, min=-self.V0[2], max=150.0 - self.V0[2])
        P.add('dv_3', 0, min=-self.V0[3], max=150.0 - self.V0[3])
        res = minimize(self.residual, P, args=(self.dx[-1], self.inv_calibration_matrix))
        dV = np.array([res.params['dv_0'].value, res.params['dv_1'].value, res.params['dv_2'].value,
                            res.params['dv_3'].value])
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.

        # There is a plus sign here instead of a minus, because I am finding and applying a change in voltage that
        # hopefully induces a dx to undo the measured dx, up to constraints on voltages.
        self.update_voltage = self.V0 + self.dV
        return self.update_voltage

    @staticmethod
    def residual(params, dx, inv_calibration_matrix):
        """
        This returns the difference between the current measured dx and the hypothetical dx induced by a voltage
        change to the piezzos.
        """
        dV = np.array([params['dv_0'].value, params['dv_1'].value, params['dv_2'].value, params['dv_3'].value])
        dx_induced = np.matmul(inv_calibration_matrix, dV)
        return np.square(dx+dx_induced)

    def calc_dx(self):
        self.dx = self.com()-self.set_pos
        return

    def store_data(self, state, IR):
        """
        STATE_MEASURE = 0
        STATE_CALIBRATE = 1
        STATE_LOCKED = 2
        STATE_ALIGN = 3
        """
        if IR:
            if state == 0:
                state = "_Measure_IR"
            elif state == 1:
                state = "_Calibrate_IR"
            elif state == 2:
                state = "_Locked_IR"
            elif state == 3:
                state = "_Align_IR"
        else:
            if state == 0:
                state = "_Measure_Vis"
            elif state == 1:
                state = "_Calibrate_Vis"
            elif state == 2:
                state = "_Locked_Vis"
            elif state == 3:
                state = "_Align_Vis"
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
        return np.nanstd(self.dx, axis=0)

    @property
    def update_voltage(self):
        return self._update_voltage

    @property
    def frequency_domain(self):
        return self._frequency_domain

    @frequency_domain.setter
    def frequency_domain(self, list):
        """
        Nu is in Hz
        list[0] is Nu for cam 1 dx[:,0:2]
        list[1] is Nu for cam 2 dx[:,2,:]
        """
        self._frequency_domain = list
        return

    @property
    def frequency_data(self):
        """
        Find the frequency data on a uniformly spaced grid from a non-uniformly sampled time domain.
        The NyQuist freq >= the (2 * minnimum dt)^-1 in the time-domain signal.
        The spacing in frequency space is defined by the time window, T.
        So, with dt, we can find the number of points we expect if we sample up to +/-1/2dt_min
        With the above, we can find Nu in 1/[unit time], which is ms here; so scale Nu by 1000 to yield Hz.
        Finally to satisfy Parseval's theorem, we need to scale the resulting frequency data by dt.
        Oh, and we scaled the time coordinate by the time window, because nfft wants -1/2 to 1/2 domain. We choose to
        define t=0 at the center of the time window. A shift in time just adds a linear phase in frequency domain.

        Set the frequency domain property and return a list of frequency data.
        """
        dt1 = self.t1[1:]-self.t1[0:-1]
        dt1_min = np.amin(dt1)
        dt2 = self.t2[1:]-self.t2[0:-1]
        dt2_min = np.amin(dt2)
        T1 = self.t1[-1]-self.t1[0]
        T2 = self.t2[-1]-self.t2[0]
        time1 = (self.t1-np.amin(self.t1))-T1/2 #t=0 at the center of the data
        time2 = (self.t2 - np.amin(self.t2)) - T2 / 2  # t=0 at the center of the data
        N1 = int(np.floor(T1/dt1_min))
        if N1 % 2 == 1:
            N1 = N1-1
        N2 = int(np.floor(T2/dt2_min))
        if N2 % 2 == 1:
            N2 = N2-1
        dNu1 = 1/T1
        dNu2 = 1/T2
        Nu1 = np.linspace(-N1/2, N1/2-1, N1)*dNu1*1000.0  # Hz
        Nu2 = np.linspace(-N2 / 2, N2 / 2 - 1, N2) * dNu2*1000.0  # Hz
        self.frequency_domain = [Nu1, Nu2]
        FT_cam1_dx = nfft.nfft_adjoint(time1/T1, self.dx[:, 0], N1)*T1/len(self.t1)
        FT_cam1_dy = nfft.nfft_adjoint(time1/T1, self.dx[:, 1], N1)*T1/len(self.t1)
        FT_cam2_dx = nfft.nfft_adjoint(time2/T2, self.dx[:, 2], N2)*T2/len(self.t2)
        FT_cam2_dy = nfft.nfft_adjoint(time2/T2, self.dx[:, 3], N2)*T2/len(self.t2)
        return [FT_cam1_dx, FT_cam1_dy, FT_cam2_dx, FT_cam2_dy]

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
        if len(self._dx) > 100:
            del self._dx[0]
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
        """Camera 1 image timestamp units of ms."""
        return np.asarray(self._t1)

    @t1.setter
    def t1(self, value):
        if len(self.t1 > 0):
            if value < self.t1[-1]-self._t1_wrapper_count*(65535+1):
                self._t1_wrapper_count += 1  # Convert to a monotonic timestamp
        self._t1.append(self._t1_wrapper_count*(65535+1)+value)
        if len(self._t1) > 100:
            del self._t1[0]
        return

    @property
    def t2(self):
        """Camera 2 image timestamp units of ms"""
        return np.asarray(self._t2)

    @t2.setter
    def t2(self, value):
        if len(self.t2 > 0):
            if value < self.t2[-1]-self._t2_wrapper_count * (65535 + 1):
                self._t2_wrapper_count += 1  # Convert to a monotonic timestamp
        self._t2.append(self._t2_wrapper_count * (65535 + 1) + value)
        if len(self._t2) > 100:
            del self._t2[0]
        return

    def load_data(self, dx, t1, t2):
        self._dx = dx
        self._t1 = t1
        self._t2 = t2
        return

    @property
    def inv_calibration_matrix(self):
        return np.linalg.inv(self.calibration_matrix)

class PIDUpdateManager(UpdateManager):

    def __init__(self):
        super().__init__()
        self._dt = None
        self._integral_ti = np.zeros(4)
        self._P = 0.5
        self._TI = 0.1
        self._TD = 0
        self._use_PID = True

    def get_update(self):
        """
        This code finds the next voltages to apply to the piezo.
        """
        if self.calibration_matrix is None or self.set_pos is None:
            raise InsufficientInformation("You must first provide a calibration matrix and a set position.")

        # Calculate dX
        # The convention here is how much has the position on the camera changed from the set point.
        self.dx = self.com()-self.set_pos
        if len(self.t1) > 1 and self._use_PID:  # PID only makes sense if there are at least 2 data points. If only one, just do P.
            # Calculate dt
            self.calc_dt()
            derivative = self.calc_derivative()
            self.calc_integral()
            #print(self.integral_ti, derivative)

            # Find dX weighted total from PID.
            dx = self.P*(self.dx[-1, :] + self.integral_ti/self.TI + self.TD*derivative)
            #print(self.integral_ti)
            #print(dx, self.dx[-1, :])
        else:
            dx = self.P*self.dx[-1, :]
        # Because of our convention for dX, the meaning of dV is how much would the voltages have changed to result
        # in the change observed in dX
        dV = np.matmul(self.calibration_matrix, dx)
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        #print(dV, self.dV)
        # place.
        # voltage to be set on the motors
        # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
        # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
        # so we subtract that dV restoring us to the old position.
        update_voltage = self.V0 - self.dV
        if np.any(update_voltage > 150) or np.any(update_voltage < 0):
            exception_message = ''
            self._use_PID = False
            self.integral_ti = np.zeros(4)
            for i in range(4):
                if update_voltage[i] < 0:
                    exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) + ' '
                if update_voltage[i] > 150:
                    exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) + ' '
            raise update_out_of_bounds(exception_message)
        else:
            self._use_PID = True
            self.update_voltage = update_voltage
            return self.update_voltage

    def fit_update(self):
        """
        When the update voltages are out of their allowed range, I want to try to find an update that minimizes
        my dx in the future step, constrained by the allowed piezo ranges. That is what this function does.

        This method works, but it does not perform well when the voltages are out of range... which is its purpose...
        SO, maybe in the future, I can try moving the target set point such that the voltages are in range but the
        set point is as close as possible. Then, maybe the updates can be stable... But not today!
        """
        #print(self.dx[-1], np.matmul(self.inv_calibration_matrix, dV_start))
        P = Parameters()
        P.add('dv_0', -self.dV[0], min=-self.V0[0], max=150.0 - self.V0[0])
        P.add('dv_1', -self.dV[1], min=-self.V0[1], max=150.0 - self.V0[1])
        P.add('dv_2', -self.dV[2], min=-self.V0[2], max=150.0 - self.V0[2])
        P.add('dv_3', -self.dV[3], min=-self.V0[3], max=150.0 - self.V0[3])

        #print(np.matmul(self.inv_calibration_matrix, np.array([P['dv_0'].value])))
        # print('Begin FIT')
        if len(self.t1) > 1:
            derivative = self.calc_derivative()
            # Bypass minimizing the PID method, i.e. False below...
            res = minimize(self.residual, P, args=(self.dx[-1], self.inv_calibration_matrix, False, derivative,
                                                   self.integral_ti, self.P, self.TI, self.TD, self.dt), max_nfev=500)
        else:
            res = minimize(self.residual, P, args=(self.dx[-1], self.inv_calibration_matrix, False), max_nfev=500)
        dV = np.array([res.params['dv_0'].value, res.params['dv_1'].value, res.params['dv_2'].value,
                            res.params['dv_3'].value])
        # the below line should be removed if miniimizing PID is done. Right now, just minimizing P term; so apply P as
        # a gain term to the found dV
        dV *= self.P
        # print(dV)
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.

        # print(self.dx[-1], np.matmul(self.inv_calibration_matrix, self.dV))

        # print(res.nfev)

        # There is a plus sign here instead of a minus, because I am finding and applying a change in voltage that
        # hopefully induces a dx to undo the measured dx, up to constraints on voltages.
        self.update_voltage = self.V0 + self.dV
        return self.update_voltage

    @staticmethod
    def residual(params, dx, inv_calibration_matrix, PID_true, derivative=None, integral=None, P=None, TI=None,
                 TD=None, dt=None):
        """
        This returns the difference between the current measured dx and the hypothetical dx induced by a voltage
        change to the piezzos where the dx in both cases are put through the PID formula.
        """
        dV = np.array([params['dv_0'].value, params['dv_1'].value, params['dv_2'].value, params['dv_3'].value])
        if PID_true:  # PID only makes sense if there are at least 2 data points. If only one, just do P.
            # Find dX weighted total from PID.
            dx_pid = P*(dx + integral/TI + TD*derivative)
            # Get the induced dx unweighted by PID
            dx_induced = np.matmul(inv_calibration_matrix, dV)
            # Find induced integral term
            increment_cam1 = dt[0] * dx_induced[0:2]  # units of pixels*s
            increment_cam2 = dt[1] * dx_induced[2:4]
            integral_induced = integral + np.concatenate([increment_cam1, increment_cam2], axis=0)
            # Find induced derivative term
            derivative_cam1 = (dx_induced[0:2] - dx[0:2]) / dt[0]
            derivative_cam2 = (dx_induced[2:4] - dx[2:4]) / dt[1]
            derivative_induced = np.concatenate([derivative_cam1, derivative_cam2], axis=0)
            # Finally, find the PID weighted dx that would be induced
            dx_pid_induced = P*(dx_induced + integral_induced/TI + TD*derivative_induced)
        else:
            dx_pid = dx
            dx_pid_induced = np.matmul(inv_calibration_matrix, dV)
        return np.square(dx_pid + dx_pid_induced)

    def calc_integral(self):
        increment_cam1 = self.dt[0]*self.dx[-1, 0:2]  # units of pixels*s
        increment_cam2 = self.dt[1]*self.dx[-1, 2:4]
        self._integral_ti += np.concatenate([increment_cam1, increment_cam2], axis=0)  # Currently using the most
        # simplistic numerical integral, may want to improve
        return

    def calc_derivative(self):
        derivative_cam1 = (self.dx[-1, 0:2] - self.dx[-2, 0:2]) / self.dt[0]
        derivative_cam2 = (self.dx[-1, 2:4] - self.dx[-2, 2:4]) / self.dt[1]
        return np.concatenate([derivative_cam1, derivative_cam2], axis=0)

    def calc_dt(self):
        dt1 = (self.t1[-1] - self.t1[-2])/1000  # Put this in s from ms
        dt2 = (self.t2[-1] - self.t2[-2])/1000
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