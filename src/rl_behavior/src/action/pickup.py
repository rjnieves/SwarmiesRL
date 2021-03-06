"""Definition of the PickupAction class.
"""

import rospy
from . import ClawFingersAction, ClawWristAction

class PickupAction(object):
  def __init__(self, swarmie_name):
    super(PickupAction, self).__init__()
    self.swarmie_name = swarmie_name
    self._sub_action_sequence = None

  def update(self, swarmie_state, elapsed_time):
    if self._sub_action_sequence is None:
      self._sub_action_sequence = [
        ClawFingersAction(self.swarmie_name, ClawFingersAction.FINGERS_OPEN),
        ClawWristAction(self.swarmie_name, ClawWristAction.WRIST_DOWN),
        ClawFingersAction(self.swarmie_name, ClawFingersAction.FINGERS_CLOSED),
        ClawWristAction(self.swarmie_name, ClawWristAction.WRIST_UP),
      ]
    next_response = None
    while self._sub_action_sequence:
      next_response = self._sub_action_sequence[0].update(swarmie_state, elapsed_time)
      if next_response is not None:
        break
      else:
        rospy.logdebug(
          'PickupAction done with {}'.format(repr(self._sub_action_sequence[0]))
        )
        del self._sub_action_sequence[0]
    if next_response is None:
      rospy.logdebug(
        'PickupAction completely done.'
      )
    return next_response

# vim: set ts=2 sw=2 expandtab:
