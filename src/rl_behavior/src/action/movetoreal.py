"""Definition of the MoveToRealAction class.
"""

import math
import numpy as np
from scipy.spatial import distance
from swarmie_msgs.msg import Skid
from . import TurnAction, DriveAction, ActionResponse

class MoveToRealAction(object):
  TURNING_STATE = 1
  DRIVING_STATE = 2
  def __init__(self, swarmie_name, target_dest):
    self.swarmie_name = swarmie_name
    self.target_dest = np.array(target_dest, dtype=np.float64)
    self._current_state = None
    self._current_action = None

  def _turn_complete_event(self):
    self._current_state = MoveToRealAction.DRIVING_STATE
    self._current_action = DriveAction(self.swarmie_name)

  def update(self, swarmie_state, elapsed_time):
    arrived = np.isclose(
      distance.euclidean(
        self.target_dest,
        swarmie_state.odom_global[0:2]
      ),
      0.0,
      atol=1e-1
    )
    if self._current_state is None:
      self._current_state = MoveToRealAction.TURNING_STATE
      target_angle = math.atan2(
        self.target_dest[1] - swarmie_state.odom_global[1],
        self.target_dest[0] - swarmie_state.odom_global[0]
      )
      self._current_action = TurnAction(self.swarmie_name, target_angle)
    next_response = self._current_action.update(swarmie_state, elapsed_time)
    if next_response is None:
      if isinstance(self._current_action, TurnAction):
        self._turn_complete_event()
        next_response = ActionResponse(Skid(left=0., right=0.))
    return None if arrived else next_response

# vim: set ts=2 sw=2 expandtab:
