"""Definition of the PathPlanning class.
"""

import heapq
import numpy as np
from scipy.spatial import distance

class PathPlanning(object):
  MOVE_NEXTROW = (1, 0)
  MOVE_PREVROW = (-1, 0)
  MOVE_NEXTCOL = (0, 1)
  MOVE_PREVCOL = (0, -1)
  ALL_MOVES = [MOVE_NEXTROW, MOVE_PREVROW, MOVE_NEXTCOL, MOVE_PREVCOL,]
  XY_MOVE_XLATE = {
    MOVE_NEXTROW: (0, -1),  # DOWN
    MOVE_PREVROW: (0, 1),   # UP
    MOVE_NEXTCOL: (1, 0),   # RIGHT
    MOVE_PREVCOL: (-1, 0),  # LEFT
  }

  @classmethod
  def translate_move(cls, matrix_move):
    return (
      cls.XY_MOVE_XLATE[matrix_move]
      if matrix_move in cls.XY_MOVE_XLATE
      else None
    )

  def __init__(self, arena):
    self.arena = arena
    self.coord_xform = arena.coord_xform
    self.reset()

  def calculate_path(self, from_grid, to_grid):
    self._planning_grid = self.arena.build_planning_grid()
    matrix_path = self._path_search(
      self.coord_xform.from_grid_to_matrix_pos(from_grid),
      self.coord_xform.from_grid_to_matrix_pos(to_grid)
    )
    return self._translate_matrix_path(matrix_path)

  def reset(self):
    self._heap_storage = []
    self._planning_grid = np.array([], dtype=np.bool)

  def _path_search(self, from_matrix_pos, to_matrix_pos):
    path = []
    visited = set()
    path_from = {}
    to_matrix_pos = tuple(to_matrix_pos)
    value = (None, tuple(from_matrix_pos), 0)
    heapq.heappush(self._heap_storage, (0, value))
    while self._heap_storage:
      (_, value) = heapq.heappop(self._heap_storage)
      (_, position, cost) = value
      if position in visited:
        continue
      visited.add(position)
      if position == to_matrix_pos:
        break
      legal_moves = self._find_legal_moves(position)
      for a_move in legal_moves:
        backwards_cost = cost + 1
        next_position = tuple(np.array(position) + np.array(a_move))
        if next_position in visited:
          continue
        forward_cost = distance.cityblock(next_position, to_matrix_pos)
        total_cost = backwards_cost + forward_cost
        new_value = (a_move, next_position, backwards_cost)
        self._update_heap(
          (
            total_cost,
            new_value
          )
        )
        if next_position in path_from:
          (_, _, curr_cost) = path_from[next_position]
          if curr_cost > total_cost:
            path_from[next_position] = (a_move, position, total_cost)
        else:
          path_from[next_position] = (a_move, position, total_cost)

    (_, path_position, _) = value
    path_action = None
    while path_position in path_from:
      path.insert(0, (path_action, path_position))
      (path_action, path_position, _) = path_from[path_position]
    path.insert(0, (path_action, path_position))
    
    return path
  
  def _find_legal_moves(self, at_matrix_pos):
    result = []
    if len(self._planning_grid) == 0:
      raise RuntimeError(
        'PathPlanning._find_legal_moves() ' +
        'precondition not met: Planning grid ready.'
      )
    bounds = np.array(self._planning_grid.shape)
    for a_move in PathPlanning.ALL_MOVES:
      new_loc = np.array(at_matrix_pos) + np.array(a_move)
      if np.any(new_loc >= bounds) or np.any(new_loc < 0):
        # Out of bounds; not legal move
        continue
      new_loc = tuple(new_loc)
      if self._planning_grid[new_loc]:
        # Obstacle in the way; not legal move
        continue
      result.append(a_move)
    return result

  def _translate_matrix_path(self, matrix_path):
    result = map(
      lambda step:
        (
          PathPlanning.translate_move(step[0]),
          tuple(self.coord_xform.from_matrix_pos_to_grid(step[1]))
        ),
      matrix_path
    )
    return result

  def _update_heap(self, new_entry):
    (new_prio, new_value) = new_entry
    for idx, existing_entry in enumerate(self._heap_storage):
      (existing_prio, existing_value) = existing_entry
      if existing_value == new_value:
        if existing_prio <= new_prio:
          break
        del self._heap_storage[idx]
        self._heap_storage.append(new_entry)
        heapq.heapify(self._heap_storage)
        break
    else:
      heapq.heappush(self._heap_storage, new_entry)
  
if __name__ == '__main__':
  from world import Arena, CoordinateTransform
  from events import SwarmieLocEvent
  coord_xform = CoordinateTransform((-7.5, 7.5), (-7.5, 7.5), 2.0)
  arena = Arena(1, coord_xform, (-4, 4), (8, 8))
  arena.swarmie_loc_update(SwarmieLocEvent(0, (-15, 15)))
  planning_grid = arena.build_planning_grid()
  print '{}'.format(arena.grid_to_str(planning_grid))
  planner = PathPlanning(arena)
  the_plan = planner.calculate_path((-15, 15), (6, 4))
  print '{}'.format(the_plan)

# vim: set ts=2 sw=2 expandtab:
