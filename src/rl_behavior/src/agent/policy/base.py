"""Definition of the BasePolicy abstract class.
"""

class BasePolicy(object):
  def __init__(self):
    pass
  
  def select_action(self, action_ranks):
    raise NotImplementedError("BasePolicy.select_action()")

  def post_train_action(self):
    pass

# vim: set ts=2 sw=2 expandtab:
