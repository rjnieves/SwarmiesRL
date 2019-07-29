"""Definition of utility functions meant to deal with robot yaw measurements.
"""

import math
import numpy as np

def yaw_wrap(yaw_value):
  yaw_value -= (2. * math.pi) if yaw_value > math.pi else 0.
  yaw_value += (2. * math.pi) if yaw_value < (-1. * math.pi) else 0.
  return yaw_value

vectorized_yaw_wrap = np.vectorize(yaw_wrap)

class YawBearing(object):
  WEST = -1.0 * math.pi
  SOUTH = -0.5 * math.pi
  EAST = 0.0
  NORTH = 0.5 * math.pi
  _BEARINGS = [WEST, SOUTH, EAST, NORTH,]

  @classmethod
  def from_yaw_value(cls, yaw_value):
    yaw_dists = vectorized_yaw_wrap(yaw_value - cls._BEARINGS)
    bearing_idx = np.argmin(np.abs(yaw_dists))
    return cls._BEARINGS[bearing_idx]

# vim: set ts=2 sw=2 expandtab:
