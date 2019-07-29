"""Definition of the SearchAction class.
"""

import math
import numpy as np
import rospy
from swarmie_msgs.msg import Skid
from . import MoveToCellAction, DriveAction, TurnAction, ActionResponse
from world import Arena
from utility import YawBearing

class SearchAction(object):
  ALPHA_QUADRANT = 'alpha'
  BRAVO_QUADRANT = 'bravo'
  CHARLIE_QUADRANT = 'charlie'
  DELTA_QUADRANT = 'delta'
  SEARCH_QUADRANTS = [
    ALPHA_QUADRANT,
    BRAVO_QUADRANT,
    CHARLIE_QUADRANT,
    DELTA_QUADRANT
  ]
  # MOVE_TO_HOME_STATE = 1
  # DRIVING_STATE = 2
  # TURNING_STATE = 3
  # LEAVING_NEST_STATE = 4

  def __init__(self, swarmie_name, arena, quadrant):
    super(SearchAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.arena = arena
    self.quadrant = quadrant
    self._search_sequence = []
    self._next_move = None
    if self.quadrant == SearchAction.ALPHA_QUADRANT:
      self._quadrant_tl = np.array([0, 0])
      self._quadrant_br = np.array([
        self.arena.nest_grid_br[0],
        self.arena.nest_grid_tl[1] - 1
      ])
      self._build_seq_col_primary()
    elif self.quadrant == SearchAction.BRAVO_QUADRANT:
      self._quadrant_tl = np.array([
        self.arena.nest_grid_br[0] + 1,
        0
      ])
      self._quadrant_br = np.array([
        self.arena.grid_dims[0] - 1,
        self.arena.nest_grid_br[1]
      ])
      self._build_seq_row_primary()
    elif self.quadrant == SearchAction.CHARLIE_QUADRANT:
      self._quadrant_tl = np.array([
        self.arena.nest_grid_tl[0],
        self.arena.nest_grid_br[1] + 1
      ])
      self._quadrant_br = np.array([
        self.arena.grid_dims[0] - 1,
        self.arena.grid_dims[1] - 1
      ])
      self._build_seq_col_primary()
    elif self.quadrant == SearchAction.DELTA_QUADRANT:
      self._quadrant_tl = np.array([
        0,
        self.arena.nest_grid_tl[1]
      ])
      self._quadrant_br = np.array([
        self.arena.nest_grid_tl[0],
        self.arena.grid_dims[1] - 1
      ])
      self._build_seq_row_primary()
    else:
      raise ValueError(
        'SearchAction invalid quadrant {}'.format(
          repr(self.quadrant)
        )
      )
  
  def _build_seq_col_primary(self):
    PRIMARY_MOVE = 0
    SECONDARY_MOVE = 1
    next_move = PRIMARY_MOVE
    next_x = self._quadrant_tl[0]
    next_y = self._quadrant_tl[1]
    while next_y <= self._quadrant_br[1]:
      self._search_sequence.append(
        MoveToCellAction(self.swarmie_name, self.arena, (next_x, next_y))
      )
      if next_move == PRIMARY_MOVE:
        next_x = self._quadrant_tl[0] if next_x == self._quadrant_br[0] else self._quadrant_br[0]
      elif next_move == SECONDARY_MOVE:
        next_y += 1
      next_move = (next_move + 1) % 2

  def _build_seq_row_primary(self):
    PRIMARY_MOVE = 0
    SECONDARY_MOVE = 1
    next_move = PRIMARY_MOVE
    next_x = self._quadrant_tl[0]
    next_y = self._quadrant_tl[1]
    while next_x <= self._quadrant_br[0]:
      self._search_sequence.append(
        MoveToCellAction(self.swarmie_name, self.arena, (next_x, next_y))
      )
      if next_move == PRIMARY_MOVE:
        next_y = self._quadrant_tl[1] if next_y == self._quadrant_br[1] else self._quadrant_br[1]
      elif next_move == SECONDARY_MOVE:
        next_x += 1
      next_move = (next_move + 1) % 2

  def update(self, swarmie_state, elapsed_time):
    if self._next_move is None:
      self._next_move = 0
    next_response = None
    while next_response is None:
      next_response = self._search_sequence[self._next_move].update(swarmie_state, elapsed_time)
      if next_response is None:
        self._next_move = (self._next_move + 1) % len(self._search_sequence)
    rospy.loginfo('SearchAction executing {}'.format(str(self._search_sequence[self._next_move])))
    return next_response

  #   self._grid_search_dir = np.array([1, 1])
  #   self._current_bearing = None
  #   self._current_state = None
  #   self._current_sub_action = None
  #   if self.quadrant == SearchAction.ALPHA_QUADRANT:
  #     self._quadrant_tl = np.array([0, 0])
  #     self._quadrant_br = np.array([
  #       self.arena.nest_grid_br[0],
  #       self.arena.nest_grid_tl[1]
  #     ])
  #     self._search_primary_axis = 0
  #     self._search_primary_bearings = {
  #       -1: YawBearing.WEST,
  #       1: YawBearing.EAST
  #     }
  #     self._search_secondary_axis = 1
  #     self._search_secondary_bearings = {
  #       -1: YawBearing.NORTH,
  #       1: YawBearing.SOUTH
  #     }
  #   elif self.quadrant == SearchAction.BRAVO_QUADRANT:
  #     self._quadrant_tl = np.array([
  #       self.arena.nest_grid_br[0],
  #       0
  #     ])
  #     self._quadrant_br = np.array([
  #       self.arena.grid_dims[0] - 1,
  #       self.arena.nest_grid_br[1]
  #     ])
  #     self._search_primary_axis = 1
  #     self._search_secondary_axis = 0
  #     self._search_primary_bearings = {
  #       -1: YawBearing.NORTH,
  #       1: YawBearing.SOUTH
  #     }
  #     self._search_secondary_axis = 1
  #     self._search_secondary_bearings = {
  #       -1: YawBearing.WEST,
  #       1: YawBearing.EAST
  #     }
  #   elif self.quadrant == SearchAction.CHARLIE_QUADRANT:
  #     self._quadrant_tl = np.array([
  #       self.arena.nest_grid_tl[0],
  #       self.arena.nest_grid_br[1]
  #     ])
  #     self._quadrant_br = np.array([
  #       self.arena.grid_dims[0] - 1,
  #       self.arena.grid_dims[1] - 1
  #     ])
  #     self._search_primary_axis = 0
  #     self._search_secondary_axis = 1
  #     self._search_primary_bearings = {
  #       -1: YawBearing.WEST,
  #       1: YawBearing.EAST
  #     }
  #     self._search_secondary_axis = 1
  #     self._search_secondary_bearings = {
  #       -1: YawBearing.NORTH,
  #       1: YawBearing.SOUTH
  #     }
  #   elif self.quadrant == SearchAction.DELTA_QUADRANT:
  #     self._quadrant_tl = np.array([
  #       0,
  #       self.arena.nest_grid_tl[1]
  #     ])
  #     self._quadrant_br = np.array([
  #       self.arena.nest_grid_tl[0],
  #       self.arena.grid_dims[1] - 1
  #     ])
  #     self._search_primary_axis = 1
  #     self._search_secondary_axis = 0
  #     self._search_primary_bearings = {
  #       -1: YawBearing.NORTH,
  #       1: YawBearing.SOUTH
  #     }
  #     self._search_secondary_axis = 1
  #     self._search_secondary_bearings = {
  #       -1: YawBearing.WEST,
  #       1: YawBearing.EAST
  #     }
  #   else:
  #     raise ValueError(
  #       'SearchAction invalid quadrant {}'.format(
  #         repr(self.quadrant)
  #       )
  #     )

  # def _position_in_zone(self, swarmie_grid_pos):
  #   return (
  #     swarmie_grid_pos[0] >= self._quadrant_tl[0] and
  #     swarmie_grid_pos[0] <= self._quadrant_br[0] and
  #     swarmie_grid_pos[1] >= self._quadrant_tl[1] and
  #     swarmie_grid_pos[1] <= self._quadrant_br[1]
  #   )

  # def _deduce_next_bearing(self, swarmie_grid_pos):
  #   candidate_pos = np.array(swarmie_grid_pos, dtype=np.int16)
  #   candidate_pos[self._search_primary_axis] += self._grid_search_dir[self._search_primary_axis]
  #   if self._position_in_zone(candidate_pos):
  #     return self._search_primary_bearings[self._grid_search_dir[self._search_primary_axis]]
  #   else:
  #     self._grid_search_dir[self._search_primary_axis] *= -1
  #     candidate_pos = np.array(swarmie_grid_pos, dtype=np.int16)
  #     candidate_pos[self._search_secondary_axis] += self._grid_search_dir[self._search_secondary_axis]
  #     if not self._position_in_zone(candidate_pos):
  #       self._grid_search_dir[self._search_secondary_axis] *= -1
  #     return self._search_secondary_bearings[self._grid_search_dir[self._search_secondary_axis]]

  # def update(self, swarmie_state, elapsed_time):
  #   swarmie_grid_pos = self.arena.coord_xform.from_real_to_grid(
  #     swarmie_state.odom_global[0:2]
  #   )
  #   self._current_bearing = YawBearing.from_yaw_value(swarmie_state.odom_current[2])
  #   if self._current_state is None:
  #     if self._position_in_zone(swarmie_grid_pos):
  #       self._current_state = SearchAction.TURNING_STATE
  #     elif self.arena.grid_loc_in_nest(swarmie_grid_pos):
  #       self._current_state = SearchAction.LEAVING_NEST_STATE
  #     else:
  #       self._current_state = SearchAction.MOVE_TO_HOME_STATE
  #   response = None
  #   if self._current_state == SearchAction.MOVE_TO_HOME_STATE:
  #     # TODO: Create MoveToCellAction if it doesn't exist, otherwise just
  #     # continue update()ing it.
  #     if self._current_sub_action is None:
  #       self._current_sub_action = MoveToCellAction(self.swarmie_name, self.arena, self._quadrant_tl)
  #     response = self._current_sub_action.update(swarmie_state, elapsed_time)
  #     if response is None:
  #       self._current_state = SearchAction.TURNING_STATE
  #       response = ActionResponse(skid=Skid(left=0., right=0.))
  #   elif self._current_state == SearchAction.TURNING_STATE:
  #     # TODO: Create TurnAction if it doesn't exist, otherwise just continue
  #     # update()ing it.
  #     if self._current_sub_action is None:
  #       new_bearing = self._deduce_next_bearing(swarmie_grid_pos)
  #       if new_bearing == self._current_bearing:
  #         self._current_state = SearchAction.DRIVING_STATE
  #         response = ActionResponse(skid=Skid(left=0., right=0.))
  #       else:
  #         self._current_sub_action = TurnAction(self.swarmie_name, new_bearing)
  #     pass
  #   elif self._current_state == SearchAction.DRIVING_STATE:
  #     # TODO: Create Drive action if it doesn't exist, otherwise just continue
  #     # update()ing it.
  #     pass
  #   elif self._current_state == SearchAction.LEAVING_NEST_STATE:
  #     # TODO: Create Drive(Reverse) action if it doesn't exist, otherwise just
  #     # continue update()ing it.
  #     pass
        
# vim: set ts=2 sw=2 expandtab:
