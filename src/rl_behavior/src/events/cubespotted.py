"""Definition of the CubeSpottedEvent class.
"""

from . import BaseEvent

class CubeSpottedEvent(BaseEvent):
  def __init__(self, cube_loc, swarmie_id):
    self.cube_loc = tuple(cube_loc)
    self.swarmie_id = int(swarmie_id)
  def __repr__(self):
    return 'CubeSpottedEvent(cube_loc={}, swarmie_id={})'.format(
      repr(self.cube_loc),
      repr(self.swarmie_id)
    )

# vim: set ts=2 sw=2 expandtab:
