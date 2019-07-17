"""Definition of the CubeDroppedEvent class.
"""

from . import BaseEvent

class CubeDroppedEvent(BaseEvent):
  def __init__(self, swarmie_id):
    super(CubeDroppedEvent, self).__init__()
    self.swarmie_id = int(swarmie_id)
  def __repr__(self):
    return 'CubeDroppedEvent(swarmie_id={})'.format(
      repr(self.swarmie_id)
    )

# vim: set ts=2 sw=2 expandtab:
