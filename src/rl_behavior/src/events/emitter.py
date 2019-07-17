"""Definition of the EventEmitter class.
"""

class EventEmitter(object):
  def __init__(self):
    self.subscription_map = {}
  
  def on_event(self, event_class, callback):
    if not event_class in self.subscription_map:
      self.subscription_map[event_class] = []
    self.subscription_map[event_class].append(
      event_class.Subscription(callback)
    )
  
  def emit(self, event):
    event_class = event.__class__
    if not event_class in self.subscription_map:
      return
    for a_subscription in self.subscription_map[event_class]:
      a_subscription(event)

# vim: set ts=2 sw=2 expandtab:
