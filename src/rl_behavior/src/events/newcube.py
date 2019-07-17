"""Definition of the NewCubeEvent class.
"""

from . import BaseEvent

class NewCubeEvent(BaseEvent):
  def __init__(self, cube_loc, swarmie_id):
    super(NewCubeEvent, self).__init__()
    self.cube_loc = tuple(cube_loc)
    self.swarmie_id = int(swarmie_id)
  def __repr__(self):
    return 'NewCubeEvent(cube_loc={}, swarmie_id={})'.format(
      repr(self.cube_loc),
      repr(self.swarmie_id)
    )

# vim: set ts=2 sw=2 expandtab:
