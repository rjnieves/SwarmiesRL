"""Definition of the MoveToCellAction class.
"""

import math
import numpy as np
import rospy
from utility import PathPlanning
from action import TurnAction, DriveAction
from swarmie_msgs.msg import Skid

class MoveToCellAction(object):
  @staticmethod
  def convert_policy_move_to_yaw(policy_move):
    if policy_move == (1, 0):
      return 0.
    elif policy_move == (-1, 0):
      return math.pi
    elif policy_move == (0, 1):
      return -1. * (math.pi / 2.)
    elif policy_move == (0, -1):
      return math.pi / 2.

  def __init__(self, swarmie_name, arena, dest_coords):
    super(MoveToCellAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.dest_coords = np.array(dest_coords, dtype=np.int16)
    if (
      np.any(np.greater_equal(self.dest_coords, arena.grid_dims)) or
      np.any(np.less(self.dest_coords, (0, 0)))
    ):
      raise ValueError('MoveToCellAction dest_coords provided out of bounds.')
    self.path_planner = PathPlanning(arena)
    self.coord_xform = arena.coord_xform
    self._policy = None
    self._current_sub_action = None
  
  def _already_turning_to(self, yaw_angle):
    return (
      self._current_sub_action is not None and
      isinstance(self._current_sub_action, TurnAction) and
      np.isclose(self._current_sub_action.target_angle, yaw_angle, atol=1e-2)
    )

  def _already_driving(self):
    return (
      self._current_sub_action is not None and
      isinstance(self._current_sub_action, DriveAction)
    )
  def update(self, swarmie_state):
    swarmie_grid_pos = self.coord_xform.from_real_to_grid(
      swarmie_state.odom_global[0:2]
    )
    swarmie_yaw = swarmie_state.odom_current[2]
    abs_swarmie_yaw = abs(swarmie_yaw)
    if abs_swarmie_yaw > 3.13 and abs_swarmie_yaw < 3.15:
      swarmie_yaw = abs_swarmie_yaw
    if self._policy is None:
      self._policy = self.path_planner.calculate_path(
        swarmie_grid_pos, self.dest_coords
      )
      rospy.loginfo(
        '{} created new path plan from {} to {}: {}'.format(
          self.swarmie_name,
          swarmie_grid_pos,
          self.dest_coords,
          self._policy
        )
      )
    next_move = None
    next_yaw = swarmie_yaw
    while self._policy:
      (next_move, at_location) = self._policy[0]
      if at_location != swarmie_grid_pos:
        del self._policy[0]
      else:
        next_yaw = MoveToCellAction.convert_policy_move_to_yaw(next_move)
        rospy.loginfo(
          '{} at {} next move is {}, yaw {}'.format(
            self.swarmie_name,
            swarmie_grid_pos,
            next_move,
            next_yaw
          )
        )
        break
    if next_move is None:
      rospy.loginfo(
        '{} path plan is all out of moves. Done.'.format(
          self.swarmie_name
        )
      )
      self._policy = None
      self._current_sub_action = None
      return None
    if not np.isclose(next_yaw, swarmie_yaw, atol=1e-2):
      # Need to turn
      if not self._already_turning_to(next_yaw):
        rospy.loginfo(
          '{} starting to turn toward {}'.format(
            self.swarmie_name,
            next_yaw
          )
        )
        self._current_sub_action = TurnAction(self.swarmie_name, next_yaw)
      else:
        rospy.loginfo(
          '{} still turning to yaw {}; currently at {}'.format(
            self.swarmie_name,
            next_yaw,
            swarmie_yaw
          )
        )
    else:
      # Forward!
      if not self._already_driving():
        rospy.loginfo(
          '{} driving forward.'.format(
            self.swarmie_name
          )
        )
        self._current_sub_action = DriveAction(self.swarmie_name)
    if self._current_sub_action is not None:
      skid_cmd = self._current_sub_action.update(swarmie_state)
      return skid_cmd or Skid(left=0., right=0.)
    else:
      self._policy = None
      self._current_sub_action = None
      return None

# vim: set ts=2 sw=2 expandtab:
