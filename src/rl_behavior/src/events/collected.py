"""Definition of the CubeCollectedEvent class.
"""

from . import BaseEvent

class CubeCollectedEvent(BaseEvent):
  def __init__(self, swarmie_id):
    super(CubeCollectedEvent, self).__init__()
    self.swarmie_id = int(swarmie_id)
  def __repr__(self):
    return 'CubeCollectedEvent(swarmie_id={})'.format(
      self.swarmie_id
    )

# vim: set ts=2 sw=2 expandtab:
