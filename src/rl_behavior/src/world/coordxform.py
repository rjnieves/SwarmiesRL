"""Definition of the CoordinateTransform class.
"""

import math
import numpy as np

class CoordinateTransform(object):
  def __init__(
    self,
    x_real_range,
    y_real_range,
    quantization
  ):
    """World to grid coordinate transformation service.

    Used to transform real-world coordinates of the swarmie arena into a grid
    specified by a quantization factor. The range of each axis is multiplied by
    the quantization factor, and the result of the operation is "floored" in
    order to yield integer results.

    :param x_real_range: Array or tuple containing the x-axis range from min to
    max.
    :type x_real_range: tuple(float, float)
    :param y_real_range: Array or tuple containing the y-axis range from min to
    max.
    :type y_real_range: tuple(float, float)
    :param quantization: Quantization factor to use.
    :type quantization: float
    :ivar quantization: Sanitized quantization factor.
    :vartype quantization: float
    :ivar x_grid_range: Array containing the min to max range of the x-axis in
    the grid.
    :vartype x_grid_range: numpy.ndarray
    :ivar y_grid_range: Array containing the min to max range of the y-axis in
    the grid.
    :vartype y_grid_range: numpy.ndarray
    :ivar grid_width: Arena width expressed as a grid cell count.
    :vartype grid_width: float
    :ivar grid_height: Arena height expressed as a grid cell count.
    :vartype grid_height: float
    """
    super(CoordinateTransform, self).__init__()
    self.quantization = math.floor(float(quantization))
    if self.quantization < 1.:
      raise ValueError('CoordinateTransform: Invalid quantization factor.')
    self.x_grid_range = np.array(
      np.floor(np.array(x_real_range) * self.quantization),
      dtype=np.int16
    )
    self.y_grid_range = np.array(
      np.floor(np.array(y_real_range) * self.quantization),
      dtype=np.int16
    )
    self.grid_width = abs(self.x_grid_range[1] - self.x_grid_range[0]) + 1
    self.grid_height = abs(self.y_grid_range[1] - self.y_grid_range[0]) + 1

  def from_real_to_grid(self, real_coords):
    """Convert from real-world coordinates to grid coordinates.

    :param real_coords: Real-world coordinates to transform.
    :type real_coords: tuple(float,float)
    :return: Converted grid coordinates.
    :rtype: numpy.ndarray
    """
    return np.array(
      np.floor(np.array(real_coords) * self.quantization),
      dtype=np.int16
    )

  def from_grid_to_matrix_pos(self, grid_coords):
    # To avoid negative values when referencing matrix rows and columns, we
    # translate and reflect the grid coordinates such that "top-left" is (0,0) 
    # instead of the more typical (xmin, ymax). Also, in accordance with NumPy's
    # default behavior, the first element in matrix position will be the row,
    # with the second being the column
    result = np.array(
      np.array(grid_coords, dtype=np.int16) +
      np.absolute([self.x_grid_range[0], self.y_grid_range[0],]),
      dtype=np.int16
    )
    result[1] = abs(result[1] - self.grid_height) - 1
    return np.flipud(result)
  
  def from_matrix_pos_to_grid(self, matrix_pos):
    result = np.array(np.flipud(matrix_pos))
    result[1] = self.grid_height - result[1] - 1
    result += np.array([self.x_grid_range[0], self.y_grid_range[0],], dtype=np.int16)
    return result

if __name__ == '__main__':
  my_xform = CoordinateTransform((-7.5, 7.5), (-7.5, 7.5), 2.)
  for orig_coord in [(-15, 15), (0, 0), (-15, -15), (15, -15), (15, 15),]:
    matrix_coord = my_xform.from_grid_to_matrix_pos(orig_coord)
    back_to_orig = my_xform.from_matrix_pos_to_grid(matrix_coord)
    print '{} to matrix {} back to {}'.format(orig_coord, tuple(matrix_coord), tuple(back_to_orig))

# vim: set ts=2 sw=2 expandtab:
