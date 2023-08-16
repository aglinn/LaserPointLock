from lmfit import minimize, Parameters
import numpy as np
from scipy.linalg import null_space
import time

calib_matrix = np.asarray(np.loadtxt(
                    'Most_Recent_Calibration.txt', dtype=float))
inv_calibration_matrix = np.linalg.inv(calib_matrix)


slow_matrix = np.empty((4, 6))
slow_matrix[:, 0:2] = inv_calibration_matrix[:, 0:2]
slow_matrix[:, 2] = inv_calibration_matrix[:, 1]
slow_matrix[:, 3:5] = inv_calibration_matrix[:, 2:]
slow_matrix[:, -1] = inv_calibration_matrix[:, -1]

N = null_space(slow_matrix)
print(np.matmul(slow_matrix, N[:, 0]))
print(N)


# Generate some fake pointing information to test with
dx = np.random.random(size=(1000, 4))


# make the residual function.
def residual(params, dx, inv_calibration_matrix):
    """
    This returns the difference between the current measured dx and the hypothetical dx induced by a voltage
    change to the piezzos.
    """
    dV = np.array([params['dv_0'].value, params['dv_1'].value, params['dv_2'].value, params['dv_3'].value])
    dx_induced = np.matmul(inv_calibration_matrix, dV)
    return np.square(dx+dx_induced)*np.array([1, 1, 100000, 100000])

# Let's imagine these updates are all from 75V:
V0 = np.array([75.0, 75.0, 75.0, 75.0])


# Make the fitting routine:
def fit_update(dx):
    global inv_calibration_matrix, V0
    """
    When the update voltages are out of their allowed range, I want to try to find an update that minimizes
    my dx in the future step, constrained by the allowed piezo ranges. That is what this function does.
    """
    P = Parameters()
    V0 = np.round(V0, decimals=3)
    P.add('dv_0', 0, min=-V0[0], max=150.0 - V0[0])
    P.add('dv_1', 0, min=-V0[1], max=150.0 - V0[1])
    P.add('dv_2', 0, min=-V0[2], max=150.0 - V0[2])
    P.add('dv_3', 0, min=-V0[3], max=150.0 - V0[3])
    res = minimize(residual, P, args=(dx, inv_calibration_matrix))
    dV = np.array([res.params['dv_0'].value, res.params['dv_1'].value, res.params['dv_2'].value,
                        res.params['dv_3'].value])
    # dV = np.round(dV, decimals=3)  # Round down on tenths decimal place, motors do not like more than 1 decimal
    # place.

    # There is a plus sign here instead of a minus, because I am finding and applying a change in voltage that
    # hopefully induces a dx to undo the measured dx, up to constraints on voltages.
    update_voltage = np.round(V0 + dV, decimals=3)
    return update_voltage

# Make the code that computes the update.
def get_update(dx):
    global calib_matrix, V0
    """
    This code finds the next voltages to apply to the piezo.
    """

    # Because of our convention for dX, the meaning of dV is how much would the voltages have changed to result
    # in the change observed in dX
    dV = np.matmul(calib_matrix, dx)
    # dV = np.round(dV, decimals=3)  # Round down on tenths decimal place, motors do not like more than 1 decimal
    # place.
    # voltage to be set on the motors
    # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
    # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
    # so we subtract that dV restoring us to the old position.
    update_voltage = np.round(V0 - dV, decimals=3)
    """if np.any(update_voltage > 150) or np.any(update_voltage < 0):
        exception_message = ''
        for i in range(4):
            if update_voltage[i] < 0:
                exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) + ' '
            if update_voltage[i] > 150:
                exception_message += 'update channel ' + str(i) + ' was ' + str(update_voltage[i]) + ' '
        print("There was an out of bounds update: " + exception_message)"""
    return update_voltage

# Now, let's loop over a bunch of dx and make sure that both methods produce the same result when they are in bounds.
passed = True
for i in range(dx.shape[0]):
    calc_v = get_update(dx[i])
    fit_v = fit_update(dx[i])
    if np.any(np.abs(calc_v - fit_v) > 0.002):
        print("The voltages were different. fit is ", fit_v, " and calculated v is ", calc_v)
        passed = False
    else:
        passed &= True

if passed:
    print("Fitting algorithm reproduces the correct update voltages while updates are in bounds.")
else:
    print("Fitting the update produced voltages different than those calculated even when dx was in bounds! Uh oh!")

# Now, I want to see if I can reproduce this behaviour, by scaling the error function in the residual.
class best_updater:

    def __init__(self, calib_matrix):
        self.calibration_matrix = calib_matrix
        self.get_dx_steps_for_find_best_udpate()
        self.increment = np.zeros(4)


    def get_dx_steps_for_find_best_udpate(self):
        """
        Figure out which dx to change if a given voltage is outside of its range. And what direction to step that dx
        component if under 0V was attempted to be applied. Always keep cam2 (focused) dxs unchanged.
        """
        dx = np.zeros(4)
        dx[0] = 1
        dV = np.matmul(self.calibration_matrix, dx)
        self._1_index_V_out_of_bounds_change_dx0 = np.argmax(np.abs(dV))
        self._1_step_dx0_V_under = np.sign(dV[self._1_index_V_out_of_bounds_change_dx0])
        dV[self._1_index_V_out_of_bounds_change_dx0] = 0
        self._2_index_V_out_of_bounds_change_dx0 = np.argmax(np.abs(dV))
        self._2_step_dx0_V_under = np.sign(dV[self._2_index_V_out_of_bounds_change_dx0])
        dx = np.zeros(4)
        dx[1] = 1
        dV = np.matmul(self.calibration_matrix, dx)
        self._1_index_V_out_of_bounds_change_dx1 = np.argmax(np.abs(dV))
        self._1_step_dx1_V_under = np.sign(dV[self._1_index_V_out_of_bounds_change_dx1])
        dV[self._1_index_V_out_of_bounds_change_dx1] = 0
        self._2_index_V_out_of_bounds_change_dx1 = np.argmax(np.abs(dV))
        self._2_step_dx1_V_under = np.sign(dV[self._2_index_V_out_of_bounds_change_dx1])

    def increment_dx(self, update_voltage):
        """
        Shift dx towards bringing voltages in range.
        """
        if update_voltage[self._1_index_V_out_of_bounds_change_dx0] < 0:
            self.update_dx[0] -= self._1_step_dx0_V_under
            self.increment[0] -= self._1_step_dx0_V_under
            ret = False
        elif update_voltage[self._1_index_V_out_of_bounds_change_dx0] > 150:
            self.update_dx[0] += self._1_step_dx0_V_under
            self.increment[0] += self._1_step_dx0_V_under
            ret = False
        elif update_voltage[self._2_index_V_out_of_bounds_change_dx0] < 0:
            self.update_dx[0] -= self._2_step_dx0_V_under
            self.increment[0] -= self._2_step_dx0_V_under
            ret = False
        elif update_voltage[self._2_index_V_out_of_bounds_change_dx0] > 150:
            self.update_dx[0] += self._2_step_dx0_V_under
            self.increment[0] += self._2_step_dx0_V_under
            ret = False
        else:
            ret = True
        if update_voltage[self._1_index_V_out_of_bounds_change_dx1] < 0:
            self.update_dx[1] -= self._1_step_dx1_V_under
            self.increment[1] -= self._1_step_dx1_V_under
            ret = False
        elif update_voltage[self._1_index_V_out_of_bounds_change_dx1] > 150:
            self.update_dx[1] += self._1_step_dx1_V_under
            self.increment[1] += self._1_step_dx1_V_under
            ret = False
        elif update_voltage[self._2_index_V_out_of_bounds_change_dx1] < 0:
            self.update_dx[1] -= self._2_step_dx1_V_under
            self.increment[1] -= self._2_step_dx1_V_under
            ret = False
        elif update_voltage[self._2_index_V_out_of_bounds_change_dx1] > 150:
            self.update_dx[1] += self._2_step_dx1_V_under
            self.increment[1] += self._2_step_dx1_V_under
            ret = False
        else:
            ret &= True
        return ret

    def find_best_update(self):
        # move the dx on unfocused cam1 until the voltages are in bounds.
        self.increment = np.zeros(4)
        while True:
            dV = np.matmul(self.calibration_matrix, self.update_dx)
            self.dV = dV
            # place.
            # voltage to be set on the motors
            # Of course, the dX is not from a voltage change but from some change in the laser; so we remove the
            # "voltage change" that would have caused dX. That is, the positions moved as though Voltages changed by dV;
            # so we subtract that dV restoring us to the old position.
            V0 = self.V0
            self.update_voltage = np.round(V0 - self.dV, decimals=3)
            if self.increment_dx(self.update_voltage):
                # No further incrementing required. Update is now in bounds.
                break
        return self.increment

# Scale dx to be large enough to go out of bounds. Then, try again.
updater = best_updater(calib_matrix)
updater.V0 = V0
print("step size of sliding method is ", updater._1_step_dx0_V_under, updater._2_step_dx0_V_under,
      updater._1_step_dx1_V_under, updater._2_step_dx1_V_under, " which corresponds to a step in voltage of ",
      np.matmul(calib_matrix, np.array([1,0,0,0])), np.matmul(calib_matrix,np.array([0,1,0,0])),
      np.matmul(calib_matrix, np.array([0,0,1,0])), np.matmul(calib_matrix,np.array([0,0,0,1])))
passed = True
dx *= 20
for i in range(dx.shape[0]):
    fit_v = fit_update(dx[i])
    fit_dv = fit_v - 75.0
    residual_dx = -1*(np.matmul(inv_calibration_matrix, fit_dv)+dx[i])
    updater.update_dx = dx[i]
    increment = updater.find_best_update()
    if np.any(np.abs(residual_dx-increment) > 1.0):
        print("The fit solution has a residual dx of ", residual_dx, "while the sliding solution incremented by ",
              increment)
        passed = False
    else:
        passed &= True

if passed:
    print("Great the fit update behaves like sliding the set position around.")

# How long are each of these methods taking?
start = time.monotonic()
for i in range(dx.shape[0]):
    updater.update_dx = dx[i]
    updater.find_best_update()
time_per_eval_sliding = (time.monotonic()-start)/dx.shape[0]
start = time.monotonic()
for i in range(dx.shape[0]):
    fit_v = fit_update(dx[i])
time_per_eval_fit = (time.monotonic()-start)/dx.shape[0]
start = time.monotonic()
for i in range(dx.shape[0]):
    calc_v = get_update(dx[i])
time_per_eval_calc = (time.monotonic()-start)/dx.shape[0]
print("The time required to get an update for calculating, sliding the target, and fitting the update are "
      "(respectively), ", time_per_eval_calc, time_per_eval_sliding, time_per_eval_fit)

"""
From this testing, I learn that fitting does work, but it is ~1000x slower and every now and then (order 1/1000), the 
update is way off, presumably because the fitting method hit its maximum eval attempts and quit with the best it had. 
So, it is better to avoid fitting if possible. 

Additionally, when the update is out of bounds, the fitting procedure produces a worse residual dx than simply stepping, 
because while the dimension that is stepped has a similar error to that of the same dimension that is fit, I am finding 
regularly that the non-stepped dimension can pickup a few pixel error, as compared to no error for stepping.
"""

"""
Now, I want to run a different test when I have two additional 'slow' motors. Let's create a ficticious inverse 
calibration matrix by copying two of the columns (i.e. assume that the slow motors are identical to the fast motors of 
two of the dimensions, which should be true to our intended design.
"""

slow_matrix = np.empty((4, 6))
slow_matrix[:, 0:2] = inv_calibration_matrix[:, 0:2]
slow_matrix[:, 2] = inv_calibration_matrix[:, 1]
slow_matrix[:, 3:5] = inv_calibration_matrix[:, 2:]
slow_matrix[:, -1] = inv_calibration_matrix[:, -1]

null_vectors = null_space(slow_matrix)

def check_existence(V: np.ndarray, null_vec):
    range = find_solution_intervals(V, null_vec)
    if range[0].max() <= range[1].min():
        return [range[0].max(), range[1].min()]
    else:
        return None

def find_solution_intervals(V, null_vec, low = 0, high = 150):
    left = (low - V) / null_vec
    right = (high - V)/null_vec
    range = np.where(left < right, [left, right], [right, left])
    return range

def get_update_5_motors(update_voltage, V0, range, null_vec, index_out_of_bounds):
    V = V0
    V[0:2] = update_voltage[0:2]
    V[3:5] = update_voltage[2:]
    if V[index_out_of_bounds] > 150:
        # Try moving the out of bounds voltage to 149 volts by walking along null vector
        ideal_scaling = check_existence(V, null_vec, 149, 149)
        if ideal_scaling is not None:
            # Then, we can move to 149 V successfully.
            return V + ideal_scaling[0] * null_vec
        else:
            # Only in this function if a solution exists, so just set the out of bounds voltage to 150
            # Just move to the middle of the range of sultions.
            return V + ((range[0] + range[1]) / 2) * null_vec
    elif update_voltage[index_out_of_bounds] < 0:
        # Try moving the out of bounds voltage to 1 volts by walking along null vector
        ideal_scaling = check_existence(V, null_vec, 1, 1)
        if ideal_scaling is not None:
            # Then, we can move to 1 V successfully.
            return V0 + ideal_scaling * null_vec
        else:
            # Only in this function if a solution exists, so just set the out of bounds voltage to 150
            # Just move to the middle of the range of sultions.
            return V0 + ((range[0] + range[1]) / 2) * null_vec