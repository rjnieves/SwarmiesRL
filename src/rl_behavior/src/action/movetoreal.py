"""Definition of the MoveToRealAction class.
"""

import math
import numpy as np
from scipy.spatial import distance
from swarmie_msgs.msg import Skid
from utility import PidLoop, yaw_wrap
from . import TurnAction, DriveAction, ActionResponse

class MoveToRealAction(object):
  TURNING_STATE = 1
  DRIVING_STATE = 2
  def __init__(self, swarmie_name, target_dest):
    self.swarmie_name = swarmie_name
    self.target_dest = np.array(target_dest, dtype=np.float64)
    self._current_state = None
    self._current_action = None
    self._last_loc = None
    self._yaw_pid = None

  def _turn_complete_event(self):
    self._current_state = MoveToRealAction.DRIVING_STATE
    self._current_action = DriveAction(self.swarmie_name)
    self._yaw_pid = PidLoop(PidLoop.Config.make_slow_yaw())

  def update(self, swarmie_state, elapsed_time):
    self._last_loc = tuple(swarmie_state.odom_global[0:2])
    arrived = np.isclose(
      distance.euclidean(
        self.target_dest,
        swarmie_state.odom_global[0:2]
      ),
      0.0,
      atol=1e-1
    )
    target_angle = math.atan2(
      self.target_dest[1] - swarmie_state.odom_global[1],
      self.target_dest[0] - swarmie_state.odom_global[0]
    )
    if self._current_state is None:
      self._current_state = MoveToRealAction.TURNING_STATE
      self._current_action = TurnAction(self.swarmie_name, target_angle)
    next_response = self._current_action.update(swarmie_state, elapsed_time)
    if next_response is None:
      if isinstance(self._current_action, TurnAction):
        self._turn_complete_event()
        next_response = ActionResponse(Skid(left=0., right=0.))
    elif self._yaw_pid and isinstance(self._current_action, DriveAction):
      yaw_output = self._yaw_pid.pid_out(
        yaw_wrap(target_angle - swarmie_state.odom_current[2]),
        target_angle
      )
      left_cmd = self._current_action.scale_wheel_cmd(
        next_response.skid.left - yaw_output
      )
      right_cmd = self._current_action.scale_wheel_cmd(
        next_response.skid.right + yaw_output
      )
      next_response = ActionResponse(
        skid=Skid(left=left_cmd, right=right_cmd)
      )
    return None if arrived else next_response

  def __str__(self):
    return 'MoveToRealAction(to={},at={})'.format(self.target_dest, self._last_loc)

# vim: set ts=2 sw=2 expandtab:
