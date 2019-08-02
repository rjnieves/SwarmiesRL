"""Definition of the EpsilonGreedyPolicy class.
"""

import random
import numpy as np
from .base import BasePolicy

# ------------------------------------------------------------------------------
# CODE ATTRIBUTION NOTICE:
# The code for the EpsilonGreedyPolicy class is primarily based on Double DQN
# example code available on GitHub at:
# https://github.com/keon/deep-q-learning
class EpsilonGreedyPolicy(BasePolicy):
  def __init__(self, epsilon, epsilon_min, epsilon_decay):
    super(EpsilonGreedyPolicy, self).__init__()
    self.epsilon = epsilon
    self.epsilon_min = epsilon_min
    self.epsilon_decay = epsilon_decay
  
  def select_action(self, action_ranks):
    if np.random.rand() <= self.epsilon:
      return random.randrange(len(action_ranks))
    else:
      return np.argmax(action_ranks)
  
  def post_train_action(self):
    # Decay the exploration factor (epsilon) after every training session
    if self.epsilon > self.epsilon_min:
      self.epsilon *= self.epsilon_decay
# END CODE ATTRIBUTION NOTICE
# ------------------------------------------------------------------------------

# vim: set ts=2 sw=2 expandtab:
