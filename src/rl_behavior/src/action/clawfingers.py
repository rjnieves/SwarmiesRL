"""Definition of the ClawFingersAction class.
"""

import math
import rospy
from std_msgs.msg import Float32
from . import ActionResponse

class ClawFingersAction(object):
  FINGERS_CLOSED = 0.0
  FINGERS_OPEN = math.pi / 2.0
  TRAVEL_TIME = rospy.Duration(secs=1)

  def __init__(self, swarmie_name, position):
    super(ClawFingersAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.position = float(position)
    self.position = min(ClawFingersAction.FINGERS_OPEN, self.position)
    self.position = max(ClawFingersAction.FINGERS_CLOSED, self.position)
    self._travel_time_left = None

  def update(self, swarmie_state, elapsed_time):
    if self._travel_time_left is None:
      # Force a copy
      self._travel_time_left = ClawFingersAction.TRAVEL_TIME - rospy.Duration()
    else:
      self._travel_time_left -= min(self._travel_time_left, elapsed_time)
    return ActionResponse(
      fingers=Float32(data=self.position)
    ) if self._travel_time_left else None

  def __repr__(self):
    return 'ClawFingersAction(swarmie_name={}, position={})'.format(
      repr(self.swarmie_name),
      repr(self.position)
    )

# vim: set ts=2 sw=2 expandtab:
