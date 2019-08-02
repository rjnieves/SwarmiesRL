"""Definition of the FetchAction class.
"""

import math
import numpy as np
from scipy.spatial import distance
import rospy
from swarmie_msgs.msg import Skid
from utility import yaw_wrap
from . import (
  MoveToCellAction,
  SweepAction,
  ApproachAction,
  PickupAction,
  DropOffAction,
  ActionResponse,
  MoveToRealAction,
  TurnAction,
  DriveAction
)

class FetchAction(object):
  MOVE_TO_TARGET_STATE = 1
  SWEEPING_SITE_STATE = 2
  APPROACHING_TARGET_STATE = 3
  PICKING_UP_TARGET_STATE = 4
  MOVE_TO_NEST_STATE = 5
  TURN_TO_NEST_CENTER_STATE = 6
  DROP_OFF_TARGET_STATE = 7
  BACK_AWAY_STATE = 8
  TURN_AWAY_STATE = 9
  BLOCK_ACQUISITION_STATES = set(
    [
      MOVE_TO_TARGET_STATE,
      SWEEPING_SITE_STATE,
      APPROACHING_TARGET_STATE,
      PICKING_UP_TARGET_STATE
    ]
  )
  BLOCK_CAPTURED_STATES = set(
    [
      MOVE_TO_NEST_STATE,
      TURN_TO_NEST_CENTER_STATE,
      DROP_OFF_TARGET_STATE,
      BACK_AWAY_STATE
    ]
  )
  DONE_STATE = 10
  NEST_DROP_OFF_REAL = {
    (False, True): (-0.4, 0.4),
    (True, True): (0.4, 0.4),
    (True, False): (0.4, -0.4),
    (False, False): (-0.4, -0.4),
  }
  BACK_AWAY_TIME = rospy.Duration(secs=3)
  TOTAL_CAPTURE_ATTEMPT_TIME = rospy.Duration(secs=8)

  def __init__(self, swarmie_name, swarmie_id, arena, rl_state_rep, tag_state):
    super(FetchAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.swarmie_id = swarmie_id
    self.arena = arena
    self.rl_state_rep = rl_state_rep
    self.target_grid_coords = rl_state_rep.nearest_cube[swarmie_id]
    self.tag_state = tag_state
    self._current_state = None
    self._current_action = None
    self._back_away_countdown = FetchAction.BACK_AWAY_TIME - rospy.Duration() # force copy
    self._capture_countdown = FetchAction.TOTAL_CAPTURE_ATTEMPT_TIME - rospy.Duration()

  def _acquisition_failed(self):
    return self.rl_state_rep.nearest_cube[self.swarmie_id] is None or not self._capture_countdown

  def _abort_acquisition(self, swarmie_state, current_state_label):
    away_yaw = yaw_wrap(swarmie_state.odom_current[2] + math.pi)
    self._current_state = FetchAction.TURN_AWAY_STATE
    self._current_action = TurnAction(self.swarmie_name, away_yaw)
    swarmie_state.cube_vanished()
    rospy.loginfo('FetchAction {} -> TURN_AWAY_STATE'.format(current_state_label))

  def _move_complete_event(self, swarmie_state, elapsed_time):
    tag_spotted = bool(self.tag_state.cube_tags)
    if self._current_state == FetchAction.MOVE_TO_TARGET_STATE:
      if self._acquisition_failed():
        self._abort_acquisition(swarmie_state, 'MOVE_TO_TARGET_STATE')
      elif tag_spotted:
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
      if self._acquisition_failed():
        self._abort_acquisition(swarmie_state, 'SWEEPING_SITE_STATE')
      elif tag_spotted:
        self._current_state = FetchAction.APPROACHING_TARGET_STATE
        rospy.loginfo('FetchAction SWEEPING_SITE_STATE -> APPROACHING_TARGET_STATE')
        self._current_action = ApproachAction(self.swarmie_name, self.tag_state)
      else:
        rospy.loginfo('FetchAction SWEEPING_SITE_STATE -> DONE_STATE')
        self._current_state = FetchAction.DONE_STATE
        swarmie_state.cube_vanished()
    else:
      rospy.logwarn('FetchAction sweep_complete_event in {} state? Abort FetchAction'.format(self._current_state))
      self._current_state = FetchAction.DONE_STATE

  def _approach_complete_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.APPROACHING_TARGET_STATE:
      if self._acquisition_failed():
        self._abort_acquisition(swarmie_state, 'APPROACHING_TARGET_STATE')
      else:
        self._current_state = FetchAction.PICKING_UP_TARGET_STATE
        rospy.loginfo('FetchAction APPROACHING_TARGET_STATE -> PICKING_UP_TARGET_STATE')
        self._current_action = PickupAction(self.swarmie_name)
    else:
      rospy.logwarn('FetchAction approach_complete_event in {} state? Abort FetchAction'.format(self._current_state))
      self._current_state = FetchAction.DONE_STATE

  def _approach_failed_event(self, swarmie_state, elapsed_time):
    self._abort_acquisition(swarmie_state, 'APPROACHING_TARGET_STATE')

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
    elif self._current_state == FetchAction.TURN_AWAY_STATE:
      self._current_state = FetchAction.DONE_STATE

  def _drop_off_complete_event(self, swarmie_state, elapsed_time):
    at_nest = distance.euclidean([0.0, 0.0], swarmie_state.odom_global[0:2]) < 0.8
    if at_nest:
      self._current_state = FetchAction.BACK_AWAY_STATE
      rospy.loginfo('FetchAction DROP_OFF_TARGET_STATE -> BACK_AWAY_STATE')
      self._current_action = DriveAction(self.swarmie_name, forward=False, speed=0.3)
      self._back_away_countdown = FetchAction.BACK_AWAY_TIME - rospy.Duration()
      rospy.loginfo('FetchAction GOOOOOOOOOOOOOOOOOOOOOOAAAAAAAL!')
    elif self._acquisition_failed():
      self._abort_acquisition(swarmie_state, 'DROP_OFF_TARGET_STATE')
    else:
      self._current_state = FetchAction.DONE_STATE
      rospy.loginfo('FetchAction DROP_OFF_TARGET_STATE -> DONE_STATE')
  
  def _drive_complete_event(self, swarmie_state, elapsed_time):
    swarmie_state.collected_cube()
    self._current_state = FetchAction.DONE_STATE
    rospy.loginfo('FetchAction DONE_STATE')

  def update(self, swarmie_state, elapsed_time):
    if self._current_state is None:
      self._current_state = FetchAction.MOVE_TO_TARGET_STATE
      rospy.loginfo('FetchAction START -> MOVE_TO_TARGET_STATE')
      self._current_action = MoveToCellAction(self.swarmie_name, self.arena, self.target_grid_coords)
    self._capture_countdown -= min(self._capture_countdown, elapsed_time)
    if self._current_action is None:
      return None
    next_response = self._current_action.update(swarmie_state, elapsed_time)
    # if self._current_state in FetchAction.BLOCK_ACQUISITION_STATES:
    #   if self.rl_state_rep.nearest_cube[self.swarmie_id] is None:
    #     next_response = None
    #     rospy.loginfo('FetchAction aborted. Cube at {} no longer available.'.format(self.target_grid_coords))
    #     swarmie_state.cube_vanished()
    #     self._current_state = FetchAction.DONE_STATE
    #     self._current_action = None
    #   if not self._capture_countdown:
    #     next_response = None
    #     rospy.loginfo('FetchAction aborted. More than 8 secs trying to capture.')
    #     self._current_state = FetchAction.DONE_STATE
    #     self._current_action = None
    #     swarmie_state.cube_vanished()
    if self._current_state == FetchAction.BACK_AWAY_STATE:
      self._back_away_countdown -= min(self._back_away_countdown, elapsed_time)
    if not self._back_away_countdown:
      next_response = None
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
      elif isinstance(self._current_action, DriveAction):
        self._drive_complete_event(swarmie_state, elapsed_time)
      if self._current_state != FetchAction.DONE_STATE:
        next_response = ActionResponse(Skid(left=0., right=0.))
    rospy.loginfo(
      'FetchAction to {} executing {}'.format(
        self.target_grid_coords,
        self._current_action
      )
    )
    return next_response

# vim: set ts=2 sw=2 expandtab:
