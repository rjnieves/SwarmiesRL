"""Definition of the SwarmieState class.
"""

import math
import numpy as np

class SwarmieState(object):
  CENTER_IDX = 0
  CURRENT_IDX = 1
  def __init__(self, swarmie_name):
    super(SwarmieState, self).__init__()
    self.swarmie_name = swarmie_name
    self._position_storage = np.zeros([2, 3])
    self._positions_populated = np.zeros([2, 3], dtype=np.bool)
    self.linear_vel = None
    self.angular_vel = None

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

  # @property
  # def map_center(self):
  #   return self._position_storage[
  #     SwarmieState.MAP_IDX,
  #     SwarmieState.CENTER_IDX
  #   ]
  
  # @map_center.setter
  # def map_center(self, new_val):
  #   self._set_position_storage(
  #     (
  #       SwarmieState.MAP_IDX,
  #       SwarmieState.CENTER_IDX
  #     ),
  #     new_val
  #   )

  # @property
  # def map_current(self):
  #   return self._position_storage[
  #     SwarmieState.MAP_IDX,
  #     SwarmieState.CURRENT_IDX
  #   ]
  
  # @map_current.setter
  # def map_current(self, new_val):
  #   self._set_position_storage(
  #     (
  #       SwarmieState.MAP_IDX,
  #       SwarmieState.CURRENT_IDX
  #     ),
  #     new_val
  #   )
  
  def initialize_centers(self, placement_radius):
    self.odom_center = np.array(
      [
        math.cos(self.odom_current[2]),
        math.sin(self.odom_current[2]),
        0.
      ]
    ) * placement_radius
    # self.map_center = np.array(
    #   [
    #     math.cos(self.map_current[2]),
    #     math.sin(self.map_current[2]),
    #     0.
    #   ]
    # ) * placement_radius
    # self.map_center += np.concatenate((self.map_current[:2], np.zeros((1,))))
  
  def local_odom_to_global(self, local_odom_pose):
    return np.array(local_odom_pose) - self.odom_center

# vim: set ts=2 sw=2 expandtab:
