"""Definition of the FetchAction class.
"""

import math
import numpy as np
from scipy.spatial import distance
import rospy
from swarmie_msgs.msg import Skid
from . import (
  MoveToCellAction,
  SweepAction,
  ApproachAction,
  PickupAction,
  DropOffAction,
  ActionResponse,
  MoveToRealAction,
  TurnAction
)

class FetchAction(object):
  MOVE_TO_TARGET_STATE = 1
  SWEEPING_SITE_STATE = 2
  APPROACHING_TARGET_STATE = 3
  PICKING_UP_TARGET_STATE = 4
  MOVE_TO_NEST_STATE = 5
  TURN_TO_NEST_CENTER_STATE = 6
  DROP_OFF_TARGET_STATE = 7
  DONE_STATE = 8
  NEST_DROP_OFF_REAL = {
    (False, True): (-0.4, 0.4),
    (True, True): (0.4, 0.4),
    (True, False): (0.4, -0.4),
    (False, False): (-0.4, -0.4),
  }

  def __init__(self, swarmie_name, arena, target_grid_coords, tag_state):
    super(FetchAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.arena = arena
    self.target_grid_coords = target_grid_coords
    self.tag_state = tag_state
    self._current_state = None
    self._current_action = None

  def _move_complete_event(self, swarmie_state, elapsed_time):
    tag_spotted = bool(self.tag_state.cube_tags)
    if self._current_state == FetchAction.MOVE_TO_TARGET_STATE:
      if tag_spotted:
        self._current_state = FetchAction.APPROACHING_TARGET_STATE
        rospy.loginfo('FetchAction MOVE_TO_TARGET_STATE -> APPROACHING_TARGET_STATE')
        self._current_action = ApproachAction(self.swarmie_name, self.tag_state)
      else:
        self._current_state = FetchAction.SWEEPING_SITE_STATE
        rospy.loginfo('FetchAction MOVE_TO_TARGET_STATE -> SWEEPING_SITE_STATE')
        self._current_action = SweepAction(self.swarmie_name, self.tag_state)
    elif self._current_state == FetchAction.MOVE_TO_NEST_STATE:
      self._current_state = FetchAction.TURN_TO_NEST_CENTER_STATE
      rospy.loginfo('FetchAction MOVE_TO_NEST_STATE -> TURN_TO_NEST_CENTER_STATE')
      target_angle = math.atan2(
        0.0 - swarmie_state.odom_global[1],
        0.0 - swarmie_state.odom_global[0]
      )
      self._current_action = TurnAction(self.swarmie_name, target_angle)
    else:
      rospy.logwarn('FetchAction move_complete_event in {} state? Abort FetchAction'.format(self._current_state))
      self._current_state = FetchAction.DONE_STATE

  def _sweep_complete_event(self, swarmie_state, elapsed_time):
    tag_spotted = bool(self.tag_state.cube_tags)
    if self._current_state == FetchAction.SWEEPING_SITE_STATE:
      if tag_spotted:
        self._current_state = FetchAction.APPROACHING_TARGET_STATE
        rospy.loginfo('FetchAction SWEEPING_SITE_STATE -> APPROACHING_TARGET_STATE')
        self._current_action = ApproachAction(self.swarmie_name, self.tag_state)
      else:
        rospy.loginfo('FetchAction SWEEPING_SITE_STATE -> DONE_STATE')
        self._current_state = FetchAction.DONE_STATE
    else:
      rospy.logwarn('FetchAction sweep_complete_event in {} state? Abort FetchAction'.format(self._current_state))
      self._current_state = FetchAction.DONE_STATE

  def _approach_complete_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.APPROACHING_TARGET_STATE:
      self._current_state = FetchAction.PICKING_UP_TARGET_STATE
      rospy.loginfo('FetchAction APPROACHING_TARGET_STATE -> PICKING_UP_TARGET_STATE')
      self._current_action = PickupAction(self.swarmie_name)
    else:
      rospy.logwarn('FetchAction approach_complete_event in {} state? Abort FetchAction'.format(self._current_state))
      self._current_state = FetchAction.DONE_STATE

  def _approach_failed_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.APPROACHING_TARGET_STATE:
      rospy.loginfo('FetchAction APPROACHING_TARGET_STATE -> DONE_STATE')
    else:
      rospy.logwarn('FetchAction approach_failed_event in {} state? Abort FetchAction'.format(self._current_state))
    self._current_state = FetchAction.DONE_STATE

  def _pickup_complete_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.PICKING_UP_TARGET_STATE:
      picked_up = bool(self.tag_state.cube_tags) and self.tag_state.cube_tags[0].is_new
      if picked_up:
        self._current_state = FetchAction.MOVE_TO_NEST_STATE
        rospy.loginfo('FetchAction PICKING_UP_TARGET_STATE -> MOVE_TO_NEST_STATE')
        quadrant_key = tuple(swarmie_state.odom_global[0:2] >= 0.0)
        nest_corner = FetchAction.NEST_DROP_OFF_REAL[quadrant_key]
        self._current_action = MoveToRealAction(
          self.swarmie_name,
          nest_corner
        )
        swarmie_state.picked_up_cube()
      else:
        self._current_state = FetchAction.DROP_OFF_TARGET_STATE
        self._current_action = DropOffAction(self.swarmie_name)
        rospy.loginfo('FetchAction PICKING_UP_TARGET_STATE -> DROP_OFF_TARGET_STATE')
    else:
      rospy.logwarn('FetchAction pickup_complete_event in {} state? Abort FetchAction'.format(self._current_state))

  def _turn_complete_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.TURN_TO_NEST_CENTER_STATE:
      self._current_state = FetchAction.DROP_OFF_TARGET_STATE
      rospy.loginfo('FetchAction TURN_TO_NEST_CENTER_STATE -> DROP_OFF_TARGET_STATE')
      self._current_action = DropOffAction(self.swarmie_name)

  def _drop_off_complete_event(self, swarmie_state, elapsed_time):
    at_nest = distance.euclidean([0.0, 0.0], swarmie_state.odom_global[0:2]) < 0.8
    if at_nest:
      rospy.loginfo('FetchAction GOOOOOOOOOOOOOOOOOOOOOOAAAAAAAL!')
      swarmie_state.collected_cube()
    self._current_state = FetchAction.DONE_STATE
    rospy.loginfo('FetchAction DONE_STATE')

  def update(self, swarmie_state, elapsed_time):
    if self._current_state is None:
      self._current_state = FetchAction.MOVE_TO_TARGET_STATE
      rospy.loginfo('FetchAction START -> MOVE_TO_TARGET_STATE')
      self._current_action = MoveToCellAction(self.swarmie_name, self.arena, self.target_grid_coords)
    next_response = self._current_action.update(swarmie_state, elapsed_time)
    if next_response is None:
      if isinstance(self._current_action, MoveToCellAction) or isinstance(self._current_action, MoveToRealAction):
        self._move_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, SweepAction):
        self._sweep_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, ApproachAction):
        if self._current_action.target_acquired:
          self._approach_complete_event(swarmie_state, elapsed_time)
        else:
          self._approach_failed_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, PickupAction):
        self._pickup_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, TurnAction):
        self._turn_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, DropOffAction):
        self._drop_off_complete_event(swarmie_state, elapsed_time)

      if self._current_state != FetchAction.DONE_STATE:
        next_response = ActionResponse(Skid(left=0., right=0.))
    return next_response

# vim: set ts=2 sw=2 expandtab:
