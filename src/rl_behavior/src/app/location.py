"""Definition of the LocationInformation class.
"""

import math
import numpy as np

class LocationInformation(object):
  ODOM_IDX = 0
  MAP_IDX = 1
  CENTER_IDX = 0
  CURRENT_IDX = 1
  def __init__(self):
    super(LocationInformation, self).__init__()
    self._storage = np.zeros([2, 2, 3])
    self._populated = np.zeros([2, 2, 3], dtype=np.bool)

  def _set_storage(self, segment_addr, new_val):
    if len(segment_addr) != 2:
      raise RuntimeError('LocationInformation set operation invalid address.')
    if len(new_val) == 2:
      new_val = tuple(new_val) + (0,)
    if len(new_val) != 3:
      raise RuntimeError('LocationInformation set operation invalid value.')
    self._storage[
      segment_addr[0],
      segment_addr[1]
    ] = np.array(new_val)
    self._populated[
      segment_addr[0],
      segment_addr[1],
      :
    ] = True

  @property
  def ready(self):
    return np.alltrue(self._populated)

  @property
  def odom_center(self):
    return self._storage[
      LocationInformation.ODOM_IDX,
      LocationInformation.CENTER_IDX
    ]
  
  @odom_center.setter
  def odom_center(self, new_val):
    self._set_storage(
      (
        LocationInformation.ODOM_IDX,
        LocationInformation.CENTER_IDX
      ),
      new_val
    )

  @property
  def odom_current(self):
    return self._storage[
      LocationInformation.ODOM_IDX,
      LocationInformation.CURRENT_IDX
    ]
  
  @odom_current.setter
  def odom_current(self, new_val):
    self._set_storage(
      (
        LocationInformation.ODOM_IDX,
        LocationInformation.CURRENT_IDX
      ),
      new_val
    )

  @property
  def odom_global(self):
    return self._storage[
      LocationInformation.ODOM_IDX,
      LocationInformation.CURRENT_IDX
    ] - self._storage[
      LocationInformation.ODOM_IDX,
      LocationInformation.CENTER_IDX
    ]

  @property
  def map_center(self):
    return self._storage[
      LocationInformation.MAP_IDX,
      LocationInformation.CENTER_IDX
    ]
  
  @map_center.setter
  def map_center(self, new_val):
    self._set_storage(
      (
        LocationInformation.MAP_IDX,
        LocationInformation.CENTER_IDX
      ),
      new_val
    )

  @property
  def map_current(self):
    return self._storage[
      LocationInformation.MAP_IDX,
      LocationInformation.CURRENT_IDX
    ]
  
  @map_current.setter
  def map_current(self, new_val):
    self._set_storage(
      (
        LocationInformation.MAP_IDX,
        LocationInformation.CURRENT_IDX
      ),
      new_val
    )
  
  def initialize_centers(self, placement_radius):
    self.odom_center = np.array(
      [
        math.cos(self.odom_current[2]),
        math.sin(self.odom_current[2]),
        0.
      ]
    ) * placement_radius
    self.map_center = np.array(
      [
        math.cos(self.map_current[2]),
        math.sin(self.map_current[2]),
        0.
      ]
    ) * placement_radius
    self.map_center += np.concatenate((self.map_current[:2], np.zeros((1,))))
  
  def local_odom_to_global(self, local_odom_pose):
    return np.array(local_odom_pose) - self.odom_center

# vim: set ts=2 sw=2 expandtab:
