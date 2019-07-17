"""Definition of the SwarmieLocEvent class.
"""

from . import BaseEvent

class SwarmieLocEvent(BaseEvent):
  def __init__(self, swarmie_id, swarmie_loc):
    super(SwarmieLocEvent, self).__init__()
    self.swarmie_id = int(swarmie_id)
    self.swarmie_loc = tuple(swarmie_loc)
  def __repr__(self):
    return 'SwarmieLocEvent(swarmie_id={}, swarmie_loc={})'.format(
      self.swarmie_id,
      self.swarmie_loc
    )

# vim: set ts=2 sw=2 expandtab:
