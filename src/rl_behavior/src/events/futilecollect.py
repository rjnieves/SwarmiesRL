"""Definition of the FutileCollectAttemptEvent class.
"""

from . import BaseEvent

class FutileCollectAttemptEvent(BaseEvent):
  def __init__(self, swarmie_id):
    super(FutileCollectAttemptEvent, self).__init__()
    self.swarmie_id = swarmie_id
  
  def __repr__(self):
    return 'FutileCollectAttemptEvent(swarmie_id={})'.format(repr(self.swarmie_id))

# vim: set ts=2 sw=2 expandtab:
