"""Definition of the ClawWristAction class.
"""

import rospy
from std_msgs.msg import Float32
from . import ActionResponse

class ClawWristAction(object):
  WRIST_DOWN = 0.7
  WRIST_UP = 0.0
  TRAVEL_TIME = rospy.Duration(secs=1)

  def __init__(self, swarmie_name, position):
    super(ClawWristAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.target_position = float(position)
    self.target_position = min(ClawWristAction.WRIST_DOWN, self.target_position)
    self.target_position = max(ClawWristAction.WRIST_UP, self.target_position)
    self._travel_time_left = None

  def update(self, swarmie_state, elapsed_time):
    if self._travel_time_left is None:
      # Force a copy
      self._travel_time_left = ClawWristAction.TRAVEL_TIME - rospy.Duration()
    else:
      self._travel_time_left -= min(self._travel_time_left, elapsed_time)
    return ActionResponse(
      wrist=Float32(data=self.target_position)
    ) if self._travel_time_left else None

  def __repr__(self):
    return 'CrawWristAction(swarmie_name={}, position={})'.format(
      repr(self.swarmie_name), repr(self.target_position)
    )

# vim: set ts=2 sw=2 expandtab:
