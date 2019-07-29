"""Definition of utility functions meant to deal with robot yaw measurements.
"""

import math
import numpy as np

def yaw_wrap(yaw_value):
  yaw_value -= (2. * math.pi) if yaw_value > math.pi else 0.
  yaw_value += (2. * math.pi) if yaw_value < (-1. * math.pi) else 0.
  return yaw_value

vectorized_yaw_wrap = np.vectorize(yaw_wrap)

# vim: set ts=2 sw=2 expandtab:
