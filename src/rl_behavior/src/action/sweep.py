"""Definition of the SweepAction class.
"""

import math
import numpy as np
import rospy
from utility import vectorized_yaw_wrap, YawBearing
from . import TurnAction

class SweepAction(object):
  ORIENTATIONS = np.array([
    YawBearing.WEST,
    YawBearing.SOUTH,
    YawBearing.EAST,
    YawBearing.NORTH
  ])
  def __init__(self, swarmie_name, tag_state):
    self.swarmie_name = swarmie_name
    self.tag_state = tag_state
    self._sub_action_sequence = None

  def update(self, swarmie_state, elapsed_time):
    if self._sub_action_sequence is None:
      # Figure out our orientation in order to derive the sweep arc
      swarmie_yaw = swarmie_state.odom_current[2]
      yaw_dists = vectorized_yaw_wrap(swarmie_yaw - SweepAction.ORIENTATIONS)
      orientation_idx = np.argmin(
        np.abs(yaw_dists)
      )
      # Calculate the indexes of the sweep positions
      sweep_idxs = np.array(
        [
          orientation_idx - 1,
          orientation_idx + 1
        ],
        dtype=np.int32
      )
      # Clean up the indexes so that they wrap around the list of orientations
      # (e.g., -1 -> 3, 4 -> 0)
      count_arr = np.array(
        [len(SweepAction.ORIENTATIONS),] * 2, dtype=np.int32
      )
      sweep_idxs = np.mod(sweep_idxs + count_arr, count_arr)
      self._sub_action_sequence = [
        TurnAction(self.swarmie_name, SweepAction.ORIENTATIONS[sweep_idxs[0]], min_skid=-30., max_skid=30.),
        TurnAction(self.swarmie_name, SweepAction.ORIENTATIONS[sweep_idxs[1]], min_skid=-30., max_skid=30.),
        TurnAction(self.swarmie_name, SweepAction.ORIENTATIONS[orientation_idx])
      ]
    next_response = None
    while self._sub_action_sequence:
      next_response = self._sub_action_sequence[0].update(swarmie_state, elapsed_time)
      if next_response is not None:
        break
      else:
        rospy.logdebug(
          'SweepAction done with {}'.format(repr(self._sub_action_sequence[0]))
        )
        del self._sub_action_sequence[0]
    if next_response is None:
      rospy.logdebug(
        'SweepAction completely done'
      )
    else:
      if self.tag_state.cube_tags:
        rospy.logdebug(
          'SweepAction found a block!'
        )
        next_response = None
    return next_response

  def __str__(self):
    return 'SweepAction'

# vim: set ts=2 sw=2 expandtab:
