"""Definition of the PidLoop class.
"""

import sys
import math
import numpy as np

class PidLoop(object):
  class Config(object):
    def __init__(self, **kwargs):
      super(PidLoop.Config, self).__init__()
      self.kp = float(kwargs.get('kp', 0.))
      self.ki = float(kwargs.get('ki', 0.))
      self.kd = float(kwargs.get('kd', 0.))
      self.sat_upper = float(kwargs.get('sat_upper', 255.))
      self.sat_lower = float(kwargs.get('sat_lower', -255.))
      self.anti_windup = float(kwargs.get('anti_windup', self.sat_upper/2.))
      self.error_hist_length = int(kwargs.get('error_hist_length', 4))
      self.always_integral = bool(kwargs.get('always_integral', False))
      self.reset_on_setpoint = bool(kwargs.get('reset_on_setpoint', True))
      self.feed_forward_multiplier = float(kwargs.get('feed_forward_multiplier', 0.))
      self.integral_dead_zone = float(kwargs.get('integral_dead_zone', 0.01))
      self.integral_error_history_length = int(kwargs.get('integral_error_history_length', 10000))
      self.integral_max = float(kwargs.get('integral_max', self.sat_upper/2.))
      self.derivative_alpha = float(kwargs.get('derivative_alpha', 0.7))

    @classmethod
    def make_slow_vel(cls):
      return cls(
        kp=100.,
        ki=8.,
        kd=1.1,
        always_integral=True,
        feed_forward_multiplier=320.
        # kp=75.,
        # ki=0.,
        # kd=0.
      )
    @classmethod
    def make_slow_yaw(cls):
      return cls(
        # kp=70.,
        # ki=16.,
        # kd=10.,
        # anti_windup=255./4.,
        # integral_max=255./6.
        kp=100.,
        ki=0.,
        kd=0.
      )
    @classmethod
    def make_turn_yaw(cls):
      return cls(
        kp=30.,
      )

  def __init__(self, config=None):
    super(PidLoop, self).__init__()
    config = PidLoop.Config() if config is None else config
    self.config = config
    self._error_list = []
    self._prev_set_point = sys.float_info.min
    self._reset_integral()
    self._hz = 10.

  def pid_out(self, calculated_error, set_point):
    calculated_error = float(calculated_error)
    set_point = float(set_point)
    if len(self._error_list) >= self.config.error_hist_length:
      self._error_list.pop()
    self._error_list.insert(0, calculated_error)
    p = i = d = 0.
    if not np.isclose(set_point, self._prev_set_point) and self.config.reset_on_setpoint:
      self._error_list = []
      self._reset_integral()
      self._prev_set_point = set_point
    feed_forward = (
      (pow(set_point, 3.) * self.config.feed_forward_multiplier) +
      (set_point * (self.config.feed_forward_multiplier / 4.6))
    )
    if not self.config.always_integral and len(self._error_list) > 1:
      sign_change = self._error_list[0] / self._error_list[1]
      if sign_change < 0:
        self._reset_integral()
        error_zero = self._error_list[0]
        self._error_list = [error_zero,]
    avg_error = calculated_error
    if len(self._error_list) >= self.config.error_hist_length:
      avg_error = np.average(self._error_list)
    # --------------------------------------------------------------------------
    # Calculate proportional
    p = self.config.kp * avg_error
    p = min(self.config.sat_upper, p)
    p = max(self.config.sat_lower, p)
    # --------------------------------------------------------------------------
    # Calculate integral
    integral_on = False
    if self._error_list and (abs(self._error_list[0]) > self.config.integral_dead_zone):
      self._integral_error_hist_array[self._step] = self._error_list[0]
      self._step += 1
      self._step %= self.config.integral_error_history_length
      integral_on = not self.config.always_integral
    integral_sum = np.sum(self._integral_error_hist_array)
    if self.config.always_integral or integral_on:
      i = self.config.ki * integral_sum
    else:
      self._reset_integral()
      i = 0.
    if abs(i) > self.config.integral_max or abs(p) > self.config.anti_windup:
      self._reset_integral()
      i = 0.
    # --------------------------------------------------------------------------
    # Calculate derivative
    if len(self._error_list) < 4:
      second_arg = 0. if len(self._error_list) < 3 else self._error_list[2]
      second_arg /= 2.
      first_arg = 0. if not self._error_list else self._error_list[0]
      first_arg += 0. if len(self._error_list) < 2 else self._error_list[1]
      first_arg /= 2.
      d = self.config.kd * (second_arg - first_arg) * self._hz
    # --------------------------------------------------------------------------
    # Calculate output
    output = p + i + d + feed_forward
    output = min(self.config.sat_upper, output)
    output = max(self.config.sat_lower, output)
    return output

  def _reset_integral(self):
    self._integral_error_hist_array = [0.,] * self.config.integral_error_history_length
    self._step = 0

if __name__ == '__main__':
  slow_vel_pid_config = PidLoop.Config()
  slow_vel_pid_config.kp = 100.
  slow_vel_pid_config.ki = 8.
  slow_vel_pid_config.kd = 1.1
  slow_vel_pid_config.sat_upper = 255.
  slow_vel_pid_config.sat_lower = -255.
  slow_vel_pid_config.anti_windup = slow_vel_pid_config.sat_upper/2.
  slow_vel_pid_config.error_hist_length = 4
  slow_vel_pid_config.always_integral = True
  slow_vel_pid_config.reset_on_setpoint = True
  slow_vel_pid_config.feed_forward_multiplier = 320.  # gives 127 pwm at 0.4 commandedspeed
  slow_vel_pid_config.integral_dead_zone = 0.01
  slow_vel_pid_config.integral_error_history_length = 10000
  slow_vel_pid_config.integral_max = slow_vel_pid_config.sat_upper/2.
  slow_vel_pid_config.derivative_alpha = 0.7

  vel_loop = PidLoop(slow_vel_pid_config)
  current_vel = 0.
  set_point = 0.5
  for _ in range(100):
    print 'current_vel={}'.format(current_vel)
    output = vel_loop.pid_out(set_point - current_vel, set_point)
    print 'pid output={}'.format(output)
    current_vel += ((output / 255.) * 0.1)

# vim: set ts=2 sw=2 expandtab:
