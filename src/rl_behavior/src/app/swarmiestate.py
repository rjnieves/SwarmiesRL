"""Definition of the SwarmieState class.
"""

import math
import numpy as np
from events import CubePickedUpEvent, CubeCollectedEvent, SwarmieLocEvent, CubeVanishedEvent

class SwarmieState(object):
  CENTER_IDX = 0
  CURRENT_IDX = 1
  LEFT_SONAR_IDX = 0
  CENTER_SONAR_IDX = 1
  RIGHT_SONAR_IDX = 2
  def __init__(self, swarmie_name, swarmie_id, coord_xform, emitter):
    super(SwarmieState, self).__init__()
    self.swarmie_name = swarmie_name
    self.swarmie_id = swarmie_id
    self.coord_xform = coord_xform
    self.emitter = emitter
    self.linear_vel = None
    self.angular_vel = None
    self.sonar_readings = np.array([5.] * 3)
    self._position_storage = np.zeros([2, 3])
    self._positions_populated = np.zeros([2, 3], dtype=np.bool)

  def _set_position_storage(self, segment_addr, new_val):
    if len(new_val) == 2:
      new_val = tuple(new_val) + (0,)
    if len(new_val) != 3:
      raise RuntimeError('SwarmieState set operation invalid value.')
    self._position_storage[
      segment_addr
    ] = np.array(new_val)
    self._positions_populated[
      segment_addr,
      :
    ] = True

  @property
  def ready(self):
    return (
      np.alltrue(self._positions_populated) and
      self.linear_vel is not None and
      self.angular_vel is not None
    )

  @property
  def odom_center(self):
    return self._position_storage[
      SwarmieState.CENTER_IDX
    ]
  
  @odom_center.setter
  def odom_center(self, new_val):
    self._set_position_storage(
      SwarmieState.CENTER_IDX,
      new_val
    )

  @property
  def odom_current(self):
    return self._position_storage[
      SwarmieState.CURRENT_IDX
    ]
  
  @odom_current.setter
  def odom_current(self, new_val):
    self._set_position_storage(
      SwarmieState.CURRENT_IDX,
      new_val
    )
    if self.ready:
      grid_loc = self.coord_xform.from_real_to_grid(
        self.odom_global[0:2]
      )
      self.emitter.emit(
        SwarmieLocEvent(
          swarmie_id=self.swarmie_id,
          swarmie_loc=grid_loc
        )
      )

  @property
  def odom_global(self):
    return np.around(
      self._position_storage[
        SwarmieState.CURRENT_IDX
      ] - self._position_storage[
        SwarmieState.CENTER_IDX
      ],
      decimals=1
    )

  def initialize_centers(self, placement_radius):
    self.odom_center = np.array(
      [
        math.cos(self.odom_current[2]),
        math.sin(self.odom_current[2]),
        0.
      ]
    ) * placement_radius
  
  def local_odom_to_global(self, local_odom_pose):
    return np.array(local_odom_pose) - self.odom_center

  def picked_up_cube(self):
    grid_loc = self.coord_xform.from_real_to_grid(
      self.odom_global[0:2]
    )
    self.emitter.emit(
      CubePickedUpEvent(
        swarmie_id=self.swarmie_id,
        cube_loc=grid_loc
      )
    )
  
  def collected_cube(self):
    self.emitter.emit(
      CubeCollectedEvent(
        swarmie_id=self.swarmie_id
      )
    )

  def cube_vanished(self):
    grid_loc = self.coord_xform.from_real_to_grid(
      self.odom_global[0:2]
    )
    self.emitter.emit(
      CubeVanishedEvent(
        swarmie_id=self.swarmie_id,
        cube_loc=grid_loc
      )
    )

# vim: set ts=2 sw=2 expandtab:
