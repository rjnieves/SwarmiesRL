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
        self.arena.grid_dims[0] - 2,
        self.arena.nest_grid_br[1]
      ])
      self._build_seq_row_primary()
    elif self.quadrant == SearchAction.CHARLIE_QUADRANT:
      self._quadrant_tl = np.array([
        self.arena.nest_grid_tl[0],
        self.arena.nest_grid_br[1] + 1
      ])
      self._quadrant_br = np.array([
        self.arena.grid_dims[0] - 2,
        self.arena.grid_dims[1] - 2
      ])
      self._build_seq_col_primary()
    elif self.quadrant == SearchAction.DELTA_QUADRANT:
      self._quadrant_tl = np.array([
        0,
        self.arena.nest_grid_tl[1]
      ])
      self._quadrant_br = np.array([
        self.arena.nest_grid_tl[0],
        self.arena.grid_dims[1] - 2
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
    rospy.loginfo('SearchAction of {} executing {}'.format(self.quadrant, str(self._search_sequence[self._next_move])))
    return next_response
        
# vim: set ts=2 sw=2 expandtab:
