import numpy as np

print('__file__={0:<35} | __name__={1:<20} | __package__={2:<20}'.format(__file__, __name__, str(__package__)))

class InsufficientInformation(Exception):

    def __init__(self, string):
        Exception.message(string)


class UpdateManager:

    def __init__(self, mode=0):
        self.calibration_matrix = None
        self.set_pos = None

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
        dV = np.matmul(self.calibration_matrix, self.dx)
        self.dV = np.floor(10 * dV) / 10  # Round down on tenths decimal place, motors do not like more than 1 decimal
        # place.
        # voltage to be set on the motors
        # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
        # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
        # so we subtract that dV restoring us to the old position.
        self.update_voltage = self.V0 - self.dV
        return self.update_voltage

    @property
    def update_voltage(self):
        return self.update_voltage

    @update_voltage.setter
    def update_voltage(self, vector):
        self.update_voltage = vector

    @property
    def dV(self):
        return self.dV

    @dV.setter
    def dV(self, vector):
        self.dV = vector

    @property
    def cam_1_com(self):
        return self.cam_1_com

    @cam_1_com.setter
    def cam_1_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 1
        """
        self.cam_1_com = vector

    @property
    def cam_2_com(self):
        return self.cam_1_com

    @cam_2_com.setter
    def cam_2_com(self, vector):
        """
        vector is a row vector (row position, column position) of COM on cam 2
        """
        self.cam_2_com = vector

    def com(self):
        return np.concatenate(self.cam_1_com, self.cam_2_com, axis=0)

    @property
    def dx(self):
        return self.dx

    @dx.setter
    def dx(self, vector):
        """
        vector defines the current displacement vector.
        vector has 4 float elements and is a row vector. This is a displacement vector along the dimensions listed below
        The convention for the vector will vector = (camera 1 row position, camera 1 column position,
        camera 2 row position, camera 2 column position).
        """
        self.dx = vector

    @property
    def V0(self):
        return self.V0

    @V0.setter
    def V0(self, vector):
        self.V0 = vector

    @property
    def calibration_matrix(self):
        return self.calibration_matrix

    @calibration_matrix.setter
    def calibration_matrix(self, matrix):
        self.calibration_matrix = matrix

    @property
    def set_pos(self):
        return self.set_pos

    @set_pos.setter
    def set_pos(self, vector):
        self.set_pos = vector

    @property
    def cam_1_img(self):
        return self.cam_1_img

    @cam_1_img.setter
    def cam_1_img(self, img):
        self.cam_1_img = img

    @property
    def cam_2_img(self):
        return self.cam_2_img

    @cam_2_img.setter
    def cam_2_img(self, img):
        self.cam_2_img = img
