"""Definition of the Arena class.
"""

import numpy as np

class Arena(object):
  """Model of the swarmie arena in grid form.

  Attributes
  ----------
  nest_grid_tl : tuple
      Top-left of the nest in grid coordinates.
  nest_grid_br : tuple
      Bottom-right of the nest in grid coordinates.
  """
  def __init__(
    self,
    swarmie_count,
    coord_xform,
    nest_grid_tl,
    nest_grid_dims
  ):
    """Primary constructor

    Parameters
    ----------
    coord_xform : world.CoordinateTransform
        Coordinate transformation service
    nest_grid_tl : tuple
        Top-left of the nest in grid coordinates.
    nest_grid_dims : tuple
        Nest size in grid dimensions.
    """
    super(Arena, self).__init__()
    self.swarmie_count = int(swarmie_count)
    self.coord_xform = coord_xform
    self.nest_grid_tl = np.array(nest_grid_tl, dtype=np.int16)
    self.nest_grid_dims = np.array(nest_grid_dims, dtype=np.int16)
    # Bottom-right of nest is a translation of nest top-left by +xdim,-ydim
    # (i.e., to the right, and down)
    self.nest_grid_br = np.array(
      nest_grid_tl + 
      (self.nest_grid_dims * np.array([1, -1], dtype=np.int16)),
      dtype=np.int16
    )
    self.arena_tl = np.array(
      [coord_xform.x_grid_range[0], coord_xform.y_grid_range[0],],
      dtype=np.int16
    )
    self.arena_dims = np.array(
      [coord_xform.grid_width, coord_xform.grid_height,],
      dtype=np.int16
    )
    # Bottom-right of the arena is a translation of arena top-left by
    # +xdim,-ydim (i.e., to the right, and down)
    self.arena_br = np.array(
      self.arena_tl +
      (self.arena_dims * np.array([1, -1], dtype=np.int16)),
      dtype=np.int16
    )
    self._swarmie_locs = np.zeros(
      (self.swarmie_count, 2),
      dtype=np.int16
    )
    self._nest_mask = np.zeros(
      (self.arena_dims[1], self.arena_dims[0],),
      dtype=np.bool
    )
    nest_matrix_tl = coord_xform.from_grid_to_matrix_pos(self.nest_grid_tl)
    nest_matrix_br = coord_xform.from_grid_to_matrix_pos(self.nest_grid_br)
    # from_grid_to_matrix_pos() returns row,col coordinate pairs
    self._nest_mask[
      nest_matrix_tl[0]:nest_matrix_br[0],
      nest_matrix_tl[1]:nest_matrix_br[1]
    ] = True

  def swarmie_loc_update(self, loc_event):
    """Update position of a swarmie robot.

    Parameters
    ----------
    loc_event : events.SwarmieLocEvent
        Event reporting the swarmie's location.
    """
    last_known_loc = self._swarmie_locs[loc_event.swarmie_id]
    swarmie_loc = np.array(loc_event.swarmie_loc, dtype=np.int16)
    if not np.all((last_known_loc, swarmie_loc)):
      # TODO: Flag location update event
      self._swarmie_locs[loc_event.swarmie_id] = swarmie_loc

  def build_planning_grid(self, include_nest=True):
    """Create 2-D representation of arena suitable for path planning.
    """
    result = np.zeros(
      self.arena_dims,
      dtype=np.bool
    )
    for a_swarmie_loc in self._swarmie_locs[:]:
      swarmie_matrix_loc = self.coord_xform.from_grid_to_matrix_pos(
        a_swarmie_loc
      )
      result[swarmie_matrix_loc[0], swarmie_matrix_loc[1]] = True
    if include_nest:
      np.logical_or(result, self._nest_mask, out=result)
    return result
  
  def loc_in_nest(self, the_loc):
    return self._nest_mask[the_loc]

  def get_swarmie_loc(self, swarmie_id):
    return self._swarmie_locs[int(swarmie_id)]
  
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
  my_xform = CoordinateTransform((-7.5, 7.5), (-7.5, 7.5), 2.0)
  my_arena = Arena(3, my_xform, (-4,4), (8, 8))
  from events import SwarmieLocEvent
  my_arena.swarmie_loc_update(
    SwarmieLocEvent(0, (-15, 15))
  )
  my_arena.swarmie_loc_update(
    SwarmieLocEvent(1, (-15, 14))
  )
  my_arena.swarmie_loc_update(
    SwarmieLocEvent(2, (-14, 15))
  )
  print '{}'.format(my_arena.grid_to_str(my_arena.build_planning_grid()))

# vim: set ts=2 sw=2 expandtab:
