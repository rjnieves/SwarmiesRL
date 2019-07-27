"""Definition of the DriveAction class.
"""

from swarmie_msgs.msg import Skid

class DriveAction(object):
  MAX_SKID_CMD = 255.
  def __init__(self, swarmie_name, forward=True, speed=0.5):
    super(DriveAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.forward = forward
    self.speed = speed
  
  def update(self, swarmie_state):
    wheel_cmd = DriveAction.MAX_SKID_CMD * self.speed
    wheel_cmd *= 1. if self.forward else -1.
    return Skid(left=wheel_cmd, right=wheel_cmd)

# vim: set ts=2 sw=2 expandtab:
