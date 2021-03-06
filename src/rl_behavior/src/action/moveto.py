"""Definition of the MoveToCellAction class.
"""

import math
import numpy as np
import rospy
from utility import PathPlanning
from action import TurnAction, DriveAction
from swarmie_msgs.msg import Skid
from utility import YawBearing, yaw_wrap
from . import ActionResponse

class CollisionImminentError(Exception):
  def __init__(self):
    super(CollisionImminentError, self).__init__()

class MoveToCellAction(object):
  @staticmethod
  def convert_policy_move_to_yaw(policy_move):
    if policy_move == (1, 0):
      # return 0.
      return YawBearing.EAST
    elif policy_move == (-1, 0):
      # return math.pi
      return YawBearing.WEST
    elif policy_move == (0, 1):
      # return -1. * (math.pi / 2.)
      return YawBearing.SOUTH
    elif policy_move == (0, -1):
      # return math.pi / 2.
      return YawBearing.NORTH
    else:
      return None

  def __init__(self, swarmie_name, arena, dest_coords):
    super(MoveToCellAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.arena = arena
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
    # self._avoid_nest = not arena.grid_loc_in_nest(dest_coords)
    self._avoid_nest = True
  
  def _already_turning_to(self, yaw_angle):
    result = (
      self._current_sub_action is not None and
      isinstance(self._current_sub_action, TurnAction)
    )
    if result:
      yaw_diff = yaw_wrap(self._current_sub_action.target_angle - yaw_angle)
      result = result and np.isclose(yaw_diff, 0., atol=2e-2)
    return result

  def _already_driving(self):
    return (
      self._current_sub_action is not None and
      isinstance(self._current_sub_action, DriveAction)
    )

  def _plan_route(self, current_grid_pos):
      basic_plan = self.path_planner.calculate_path(
        current_grid_pos,
        tuple(self.dest_coords)
      )
      rospy.logdebug(
        '{} created new path plan from {} to {}: {}'.format(
          self.swarmie_name,
          current_grid_pos,
          self.dest_coords,
          basic_plan
        )
      )
      self._policy = dict(
        (loc, move) for move, loc in basic_plan
      )

  def update(self, swarmie_state, elapsed_time):
    swarmie_grid_pos = self.coord_xform.from_real_to_grid(
      swarmie_state.odom_global[0:2]
    )
    swarmie_yaw = swarmie_state.odom_current[2]
    if self._policy is None:
      self._plan_route(swarmie_grid_pos)
    next_move = None
    error_countdown = 3
    next_yaw = swarmie_yaw
    while error_countdown > 0:
      try:
        next_move = self._policy[swarmie_grid_pos]
        next_yaw = MoveToCellAction.convert_policy_move_to_yaw(next_move)
        if next_move is not None:
          future_pos = np.array(swarmie_grid_pos, dtype=np.int16) + np.array(next_move, dtype=np.int16)
          if self.arena.grid_cell_occupied(future_pos):
            rospy.logwarn(
              '{} about to encroach onto another swarmie\'s space!'.format(
                self.swarmie_name
              )
            )
            raise CollisionImminentError
        break
      except KeyError:
        self._plan_route(swarmie_grid_pos)
        error_countdown -= min(error_countdown, 0)
      except CollisionImminentError:
        self._plan_route(swarmie_grid_pos)
        error_countdown -= min(error_countdown, 0)
    rospy.logdebug(
      '{} at {} next move is {}, yaw {}'.format(
        self.swarmie_name,
        swarmie_grid_pos,
        next_move,
        next_yaw
      )
    )
    if next_move is None:
      rospy.logdebug(
        '{} path plan is all out of moves. Done.'.format(
          self.swarmie_name
        )
      )
      self._policy = None
      self._current_sub_action = None
      return None
    yaw_diff = yaw_wrap(next_yaw - swarmie_yaw)
    if not np.isclose(yaw_diff, 0., atol=2e-2):
      # Need to turn
      if not self._already_turning_to(next_yaw):
        rospy.logdebug(
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
        rospy.logdebug(
          '{} driving forward.'.format(
            self.swarmie_name
          )
        )
        self._current_sub_action = DriveAction(self.swarmie_name)
    if self._current_sub_action is not None:
      response = self._current_sub_action.update(swarmie_state, elapsed_time)
      return response or ActionResponse(skid=Skid(left=0, right=0))
    else:
      self._policy = None
      self._current_sub_action = None
      return None

  def __str__(self):
    return 'MoveToCellAction({})'.format(self.dest_coords)

# vim: set ts=2 sw=2 expandtab:
