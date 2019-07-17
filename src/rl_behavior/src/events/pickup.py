"""Definition for the CubePickedUpEvent class.
"""

from . import BaseEvent

class CubePickedUpEvent(BaseEvent):
  def __init__(self, cube_loc, swarmie_id):
    super(CubePickedUpEvent, self).__init__()
    self.cube_loc = tuple(cube_loc)
    self.swarmie_id = int(swarmie_id)
  def __repr__(self):
    return 'CubePickedUpEvent(cube_loc={}, swarmie_id={})'.format(
      repr(self.cube_loc),
      repr(self.swarmie_id)
    )

# vim: set ts=2 sw=2 expandtab:
