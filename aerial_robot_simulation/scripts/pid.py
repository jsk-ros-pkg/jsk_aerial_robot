import time
import warnings
from aerial_robot_msgs.msg import Pid

from simple_pid import *

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value

try:
    # get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time
    warnings.warn('time.monotonic() not available in python < 3.3, using time.time() as fallback')

class PI_D(PID):
    def __call__(self, input_, vel_ = 0.0, error_ = None, dt=None):
        """
        Update the PID controller.
        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).
        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = _current_time()
        if dt is None:
            dt = now - self._last_time if now - self._last_time else 1e-16
        elif dt <= 0:
            raise ValueError('dt has nonpositive value {}. Must be positive.'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # only update every sample_time seconds
            return self._last_output

        # compute error terms
        if error_ is None:
            error = self.setpoint - input_
        else:
            error = error_

        d_input = input_ - (self._last_input if self._last_input is not None else input_)

        self._state = input_
        self._vel_state = vel_
        self._error = error

        # compute the proportional term
        if not self.proportional_on_measurement:
            # regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # avoid integral windup

        self._derivative = -self.Kd * vel_

        # compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        self._output = output

        # keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output

    def getDebugMsg(self):
        msg = Pid()
        msg.target_p = self.setpoint
        msg.err_p = self._error
        msg.target_d = 0
        msg.err_d = 0 - self._vel_state
        msg.total.append(self._output)
        msg.p_term.append(self._proportional)
        msg.i_term.append(self._integral)
        msg.d_term.append(self._derivative)

        return msg
