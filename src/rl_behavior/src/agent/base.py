"""Definition of the BaseAgent abstract class.
"""

class BaseAgent(object):
  def __init__(self, policy, state_size, action_size):
    self.policy = policy
    self.state_size = state_size
    self.action_size = action_size

  def act(self, state):
    raise NotImplementedError("BaseAgent.act()")
  
  def remember(self, state, action, reward, next_state, done):
    pass
  
  def learn(self):
    raise NotImplementedError("BaseAgent.learn()")

# vim: set ts=2 sw=2 expandtab:
