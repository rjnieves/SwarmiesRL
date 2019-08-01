"""Definition of the CubeVanishedEvent class.
"""

from . import BaseEvent

class CubeVanishedEvent(BaseEvent):
  def __init__(self, swarmie_id, cube_loc):
    super(CubeVanishedEvent, self).__init__()
    self.swarmie_id = int(swarmie_id)
    self.cube_loc = tuple(cube_loc)

  def __repr__(self):
    return 'CubeVanishedEvent(swarmie_id={}, cube_loc={})'.format(
      repr(self.swarmie_id),
      repr(self.cube_loc)
    )

# vim: set ts=2 sw=2 expandtab:
