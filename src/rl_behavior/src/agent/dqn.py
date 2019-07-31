"""Definition of the DqnAgent class.
"""

import random
from collections import deque
import numpy as np
import tensorflow as tf
from keras.models import Sequential, load_model
from keras.layers import Dense
from keras.optimizers import Adam
from keras import backend as K
from keras.utils.generic_utils import get_custom_objects
from .base import BaseAgent

def _huber_loss(y_true, y_pred, clip_delta=1.0):
  error = y_true - y_pred
  cond = K.abs(error) <= clip_delta

  squared_loss = 0.5 * K.square(error)
  quadratic_loss = 0.5 * K.square(clip_delta) + clip_delta * (K.abs(error) - clip_delta)

  return K.mean(tf.where(cond, squared_loss, quadratic_loss))

class DqnAgent(BaseAgent):
  def __init__(self, policy, state_size, action_size, model_file_path):
    super(DqnAgent, self).__init__(policy, state_size, action_size)
    get_custom_objects().update({'_huber_loss': _huber_loss})
    self.model = load_model(model_file_path)

  def act(self, state):
    batch = np.reshape(state, [1, self.state_size])
    result = self.model.predict(batch)
    action_ranks = result[0]
    return self.policy.select_action(action_ranks)

# vim: set ts=2 sw=2 expandtab:
