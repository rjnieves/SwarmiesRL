"""Definition of the AxisQuantization class.
"""

import numpy as np

class AxisQuantization(object):
  def __init__(self, extents, bin_count):
    super(AxisQuantization, self).__init__()
    self.extents = np.array(extents)
    if self.extents.shape != (2,):
      raise ValueError('AxisQuantization extents not a 1-D, 2-element tuple.')
    self._ordered_extents = np.sort(self.extents)
    bin_count = int(bin_count)
    if bin_count < 1:
      raise ValueError('AxisQuantization bin count < 1.')
    self._quant_edges = np.linspace(
      extents[0],
      extents[1],
      num=bin_count
    )
    self._digitize_to_right = self.extents[0] > self.extents[1]
  
  @property
  def axis_real_dim(self):
    return np.diff(self._ordered_extents)[0]

  @property
  def axis_quant_dim(self):
    return self._quant_edges.shape[0]

  def quantize(self, real_value):
    real_value = min(real_value, self._ordered_extents[1])
    real_value = max(real_value, self._ordered_extents[0])
    result = np.digitize(
      real_value,
      self._quant_edges,
      right=self._digitize_to_right
    )
    return result - 1

  def edge_value(self, edge_idx):
    return self._quant_edges[edge_idx]

if __name__ == '__main__':
  x_quant = AxisQuantization([-7.5, 7.5], 30)
  print 'X-Axis ---------------------------------------------------------------'
  for a_val in [-2.3, -2.2, 2.3, 2.2, 1.7, 1.5, 1.8, 0., -7.5, 7.5]:
    quant_val = x_quant.quantize(a_val)
    print '{} -> quantized={}, slot={}'.format(
      a_val,
      quant_val,
      x_quant.edge_value(quant_val)
    )
  print 'Y-Axis ---------------------------------------------------------------'
  y_quant = AxisQuantization([7.5, -7.5], 30)
  for a_val in [-2.3, -2.2, 2.3, 2.2, 1.7, 1.5, 1.8, 0., -7.5, 7.5]:
    quant_val = y_quant.quantize(a_val)
    print '{} -> quantized={}, slot={}'.format(
      a_val,
      quant_val,
      y_quant.edge_value(quant_val)
    )
  print 'Grid -----------------------------------------------------------------'
  print '({}, {})'.format(x_quant.axis_quant_dim, y_quant.axis_quant_dim)
  print 'Real -----------------------------------------------------------------'
  print '({}, {})'.format(x_quant.axis_real_dim, y_quant.axis_real_dim)

# vim: set ts=2 sw=2 expandtab:
