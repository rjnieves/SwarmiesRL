"""Definition of the CoordinateTransform class.
"""

import math
import numpy as np
from axisquant import AxisQuantization

class CoordinateTransform(object):
  def __init__(
    self,
    x_real_range,
    y_real_range,
    quantization
  ):
    """World to grid coordinate transformation service.

    Used to transform real-world coordinates of the swarmie arena into a grid
    specified by a quantization values. In the resulting grid, the row and
    column dimensions will be directly related to the quantization values.

    :param x_real_range: Array or tuple containing the x-axis range from min to
    max.
    :type x_real_range: tuple(float, float)
    :param y_real_range: Array or tuple containing the y-axis range from min to
    max.
    :type y_real_range: tuple(float, float)
    :param quantization: Quantization values to use.
    :type quantization: tuple(int, int)
    """
    super(CoordinateTransform, self).__init__()
    quantization = np.array(quantization, dtype=np.int16)
    if quantization.shape != (2,):
      raise ValueError('CoordinateTransform quantization specification not a 1-D, 2-element tuple.')
    x_real_range = np.array(x_real_range)
    if x_real_range.shape != (2,):
      raise ValueError('CoordinateTransform X-Axis range not a 1-D, 2-element tuple.')
    if np.allclose(np.diff(x_real_range), [0.0,]):
      raise ValueError('CoordinateTransform X-Axis range 0.0 or too small.')
    y_real_range = np.array(y_real_range)
    if y_real_range.shape != (2,):
      raise ValueError('CoordinateTransform Y-Axis range not a 1-D, 2-element tuple.')
    if np.allclose(np.diff(y_real_range), [0.0,]):
      raise ValueError('CoordinateTransform Y-Axis range 0.0 or too small.')
    # List the Y-Axis extents from high to low so that the grid coordinates list
    # the highest Y-Axis value as row 0 and lowest Y-Axis value as row n-1
    if np.diff(y_real_range)[0] > 0.:
      y_real_range = np.roll(y_real_range, 1)
    self.x_quant = AxisQuantization(x_real_range, quantization[0])
    self.y_quant = AxisQuantization(y_real_range, quantization[1])

  def from_real_to_grid(self, real_coords):
    """Convert from real-world coordinates to grid coordinates.

    :param real_coords: Real-world coordinates to transform.
    :type real_coords: tuple(float,float)
    :return: Converted grid coordinates.
    :rtype: numpy.ndarray
    """
    if len(real_coords) != 2:
      raise ValueError('CoordinateTransform.from_real_to_grid() real_coords not a 1-D, 2-element tuple.')
    return (
      self.x_quant.quantize(real_coords[0]),
      self.y_quant.quantize(real_coords[1])
    )

if __name__ == '__main__':
  my_xform = CoordinateTransform((-7.5, 7.5), (-7.5, 7.5), (30, 30,))
  for orig_coord in [(-7.5, 7.5), (0., 0.), (-7.5, -7.5), (7.5, -7.5), (7.5, 7.5),]:
    matrix_coord = my_xform.from_real_to_grid(orig_coord)
    print '{} to matrix {}'.format(orig_coord, matrix_coord)

# vim: set ts=2 sw=2 expandtab:
