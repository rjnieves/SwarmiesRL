"""Definition of the TurnAction class.
"""

import math
import numpy as np
import rospy
from swarmie_msgs.msg import Skid
from app import PidLoop

class TurnAction(object):
  MIN_SKID_CMD = -180.
  MAX_SKID_CMD = 180.

  def __init__(self, swarmie_name, target_angle):
    super(TurnAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.yaw_pid = None
    self.target_angle = target_angle
    abs_target_angle = abs(self.target_angle)
    if abs_target_angle > 3.13 and abs_target_angle < 3.15:
      self.target_angle = abs_target_angle

  def update(self, swarmie_state):
    yaw_current = swarmie_state.odom_current[2]
    abs_yaw_current = abs(yaw_current)
    if abs_yaw_current > 3.13 and abs_yaw_current < 3.15:
      # In that magic pi/-pi discontinuity, always stay positive
      yaw_current = abs_yaw_current
    if np.isclose(yaw_current, self.target_angle, atol=1e-2):
      rospy.loginfo(
        '{} yaw {} close enough to requested angle {}'.format(
          self.swarmie_name,
          yaw_current,
          self.target_angle
        )
      )
      return None
    if self.yaw_pid is None:
      self.yaw_pid = PidLoop(PidLoop.Config.make_turn_yaw())
    yaw_error = self.target_angle - yaw_current
    yaw_error -= (2. * math.pi) if yaw_error > math.pi else 0.
    yaw_error += (2. * math.pi) if yaw_error < (-1. * math.pi) else 0.
    rospy.loginfo(
      '{} yaw set point is ({}), current is ({}), making an error of ({})'.format(
        self.swarmie_name,
        self.target_angle,
        yaw_current,
        yaw_error
      )
    )
    yaw_output = self.yaw_pid.pid_out(yaw_error, self.target_angle)
    rospy.logdebug(
      '{} PID output for yaw ({})'.format(
        self.swarmie_name,
        yaw_output
      )
    )
    left_cmd = 0. - yaw_output
    left_cmd = min(TurnAction.MAX_SKID_CMD, left_cmd)
    left_cmd = max(TurnAction.MIN_SKID_CMD, left_cmd)
    right_cmd = 0. + yaw_output
    right_cmd = min(TurnAction.MAX_SKID_CMD, right_cmd)
    right_cmd = max(TurnAction.MIN_SKID_CMD, right_cmd)
    return Skid(left=left_cmd, right=right_cmd)

# vim: set ts=2 sw=2 expandtab:
