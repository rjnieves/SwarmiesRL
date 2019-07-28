"""Definition of the ClawWristAction class.
"""

from std_msgs.msg import Float32

class ClawWristAction(object):
  WRIST_DOWN = 0.7
  WRIST_UP = 0.0

  def __init__(self, swarmie_name, position):
    super(ClawWristAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.target_position = float(position)
    self.target_position = min(ClawWristAction.WRIST_DOWN, self.target_position)
    self.target_position = max(ClawWristAction.WRIST_UP, self.target_position)

  def update(self, swarmie_state):
    return Float32(data=self.target_position)

# vim: set ts=2 sw=2 expandtab:
