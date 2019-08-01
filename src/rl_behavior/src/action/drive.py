"""Definition of the DriveAction class.
"""

from swarmie_msgs.msg import Skid
from . import ActionResponse

class DriveAction(object):
  MAX_SKID_CMD = 255.
  def __init__(self, swarmie_name, forward=True, speed=0.5):
    super(DriveAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.forward = forward
    self.speed = speed
  
  def scale_wheel_cmd(self, raw_cmd):
    upper_bound = DriveAction.MAX_SKID_CMD * self.speed
    lower_bound = -1.0 * upper_bound
    raw_cmd = min(upper_bound, raw_cmd)
    raw_cmd = max(lower_bound, raw_cmd)
    return raw_cmd

  def update(self, swarmie_state, elapsed_time):
    wheel_cmd = DriveAction.MAX_SKID_CMD
    wheel_cmd *= 1. if self.forward else -1.
    wheel_cmd = self.scale_wheel_cmd(wheel_cmd)
    return ActionResponse(
      Skid(left=wheel_cmd, right=wheel_cmd)
    )

# vim: set ts=2 sw=2 expandtab:
