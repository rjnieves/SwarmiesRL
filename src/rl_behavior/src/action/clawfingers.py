"""Definition of the ClawFingersAction class.
"""

import math
from std_msgs.msg import Float32

class ClawFingersAction(object):
  FINGERS_CLOSED = 0.0
  FINGERS_OPEN = math.pi / 2.0

  def __init__(self, swarmie_name, position):
    super(ClawFingersAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.position = float(position)
    self.position = min(ClawFingersAction.FINGERS_OPEN, self.position)
    self.position = max(ClawFingersAction.FINGERS_CLOSED, self.position)

  def update(self, swarmie_state):
    return Float32(data=self.position)

# vim: set ts=2 sw=2 expandtab:
