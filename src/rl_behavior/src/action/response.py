"""Definition of the ActionResponse class.
"""

class ActionResponse(object):
  def __init__(self, skid=None, wrist=None, fingers=None):
    super(ActionResponse, self).__init__()
    self.skid = skid
    self.wrist = wrist
    self.fingers = fingers

# vim: set ts=2 sw=2 expandtab:
