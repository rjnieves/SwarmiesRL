"""Definition of the BaseEvent abstract class.
"""

class BaseEvent(object):
  class Subscription(object):
    def __init__(self, callback):
      super(BaseEvent.Subscription, self).__init__()
      self.callback = callback
    def __call__(self, event):
      self.callback(event)
  def __init__(self):
    super(BaseEvent, self).__init__()

# vim: set ts=2 sw=2 expandtab:
