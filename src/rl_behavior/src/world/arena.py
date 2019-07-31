"""Definition of the Arena class.
"""

import numpy as np
from events import SwarmieLocEvent


class Arena(object):
  """Model of the swarmie arena in grid form.

  Attributes
  ----------
  nest_real_tl : tuple
      Top-left of the nest in real coordinates.
  nest_real_br : tuple
      Bottom-right of the nest in real coordinates.
  """
  def __init__(
    self,
    swarmie_count,
    coord_xform,
    nest_real_tl,
    nest_real_dims,
    emitter
  ):
    """Primary constructor

    Parameters
    ----------
    swarmie_count : int
        Number of swarmie robots participating
    coord_xform : world.CoordinateTransform
        Coordinate transformation service
    nest_real_tl : tuple(float, float)
        Top-left of the nest in real coordinates.
    nest_real_dims : tuple(float, float)
        Nest size in real dimensions.
    emitter : events.EventEmitter
        Local event emitter
    """
    super(Arena, self).__init__()
    self.swarmie_count = int(swarmie_count)
    self.coord_xform = coord_xform
    self.emitter = emitter
    self.emitter.on_event(SwarmieLocEvent, self.swarmie_loc_update)
    self.grid_dims = (
      self.coord_xform.x_quant.axis_quant_dim,
      self.coord_xform.y_quant.axis_quant_dim
    )
    self.nest_real_tl = np.array(nest_real_tl)
    self.nest_real_dims = np.array(nest_real_dims)
    # Bottom-right of nest is a translation of nest top-left by +xdim,-ydim
    # (i.e., to the right, and down)
    self.nest_real_br = np.array(
      self.nest_real_tl + 
      (self.nest_real_dims * np.array([1., -1.]))
    )
    self._swarmie_grid_locs = np.zeros(
      (self.swarmie_count, 2),
      dtype=np.int16
    )
    self._nest_mask = np.zeros(
      self.grid_dims,
      dtype=np.bool
    )
    self.nest_grid_tl = coord_xform.from_real_to_grid(self.nest_real_tl)
    self.nest_grid_br = coord_xform.from_real_to_grid(self.nest_real_br)
    # from_grid_to_matrix_pos() returns row,col coordinate pairs
    self._nest_mask[
      self.nest_grid_tl[0]:self.nest_grid_br[0],
      self.nest_grid_tl[1]:self.nest_grid_br[1]
    ] = True

  def swarmie_loc_update(self, loc_event):
    """Update position of a swarmie robot.

    Parameters
    ----------
    loc_event : events.SwarmieLocEvent
        Event reporting the swarmie's location.
    """
    last_known_grid_loc = self._swarmie_grid_locs[loc_event.swarmie_id]
    swarmie_grid_loc = np.array(
      loc_event.swarmie_loc, dtype=np.int16
    )
    if not np.all(last_known_grid_loc == swarmie_grid_loc):
      # TODO: Flag location update event
      self._swarmie_grid_locs[loc_event.swarmie_id] = swarmie_grid_loc

  def grid_cell_occupied(self, grid_coords):
    result = False
    grid_coords = np.array(grid_coords, dtype=np.int16)
    for swarmie_grid_loc in self._swarmie_grid_locs[:]:
      result = result or np.all(swarmie_grid_loc == grid_coords)
    return result

  def build_planning_grid(self, include_nest=True):
    """Create 2-D representation of arena suitable for path planning.
    """
    result = np.zeros(
      self.grid_dims,
      dtype=np.bool
    )
    for a_swarmie_loc in self._swarmie_grid_locs[:]:
      result[a_swarmie_loc[0], a_swarmie_loc[1]] = True
    if include_nest:
      np.logical_or(result, self._nest_mask, out=result)
    return result

  def real_loc_in_nest(self, real_coords):
    grid_coords = self.coord_xform.from_real_to_grid(real_coords)
    return self.grid_loc_in_nest(grid_coords)

  def grid_loc_in_nest(self, grid_coords):
    return self._nest_mask[grid_coords]

  def grid_to_str(self, grid):
    return '\n'.join(
      map(
        lambda row:
          ''.join(
            map(
              lambda cell:
                '*' if cell else '.',
              row
            )
          ),
        grid
      )
    )


if __name__ == '__main__':
  from coordxform import CoordinateTransform
  from events import EventEmitter
  my_emitter = EventEmitter()
  my_xform = CoordinateTransform((-7.5, 7.5), (-7.5, 7.5), (30, 30,))
  my_arena = Arena(3, my_xform, (-2.0, 2.0), (4.0, 4.0), my_emitter)
  from events import SwarmieLocEvent
  my_arena.swarmie_loc_update(
    SwarmieLocEvent(0, (-7.5, 7.5))
  )
  my_arena.swarmie_loc_update(
    SwarmieLocEvent(1, (-7.5, 6.5))
  )
  my_arena.swarmie_loc_update(
    SwarmieLocEvent(2, (-6.9, 7.5))
  )
  print '{}'.format(my_arena.grid_to_str(my_arena.build_planning_grid()))

# vim: set ts=2 sw=2 expandtab:
