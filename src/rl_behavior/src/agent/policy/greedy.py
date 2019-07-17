"""Definition of the GreedyPolicy class.
"""

import numpy as np
from .base import BasePolicy

class GreedyPolicy(BasePolicy):
  def __init__(self):
    super(GreedyPolicy, self).__init__()
  
  def select_action(self, action_ranks):
    return np.argmax(action_ranks)

# vim: set ts=2 sw=2 expandtab:
