"""Definition of the DqnAgent class.
"""

import random
from collections import deque
import numpy as np
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from keras import backend as K

from .base import BaseAgent

class DqnAgent(BaseAgent):
  def __init__(self, policy, state_size, action_size, minibatch_size):
    super(DqnAgent, self).__init__(policy, state_size, action_size)
    self.minibatch_size = minibatch_size
    self.memory = deque(maxlen=2000)
    self.gamma = 0.95
    self.learning_rate = 0.001
    self.model = self._build_model()
    self.target_model = self._build_model()
    self.update_target_model()
  
  def _huber_loss(self, y_true, y_pred, clip_delta=1.0):
    error = y_true - y_pred
    cond = K.abs(error) <= clip_delta

    squared_loss = 0.5 * K.square(error)
    quadratic_loss = 0.5 * K.square(clip_delta) + clip_delta * (K.abs(error) - clip_delta)

    return K.mean(tf.where(cond, squared_loss, quadratic_loss))

  def _build_model(self):
    model = Sequential()
    model.add(Dense(48, input_dim=self.state_size, activation='relu'))
    model.add(Dense(48, activation='relu'))
    model.add(Dense(self.action_size, activation='linear'))
    model.compile(
      loss=self._huber_loss,
      optimizer=Adam(
        lr=self.learning_rate
      )
    )
    return model

  def update_target_model(self):
    self.target_model.set_weights(self.model.get_weights())

  def act(self, state):
    batch = np.reshape(state, [1, self.state_size])
    result = self.model.predict(batch)
    action_ranks = result[0]
    return self.policy.select_action(action_ranks)
  
  def remember(self, state, action, reward, next_state, done):
    self.memory.append((state, action, reward, next_state, done))
  
  def learn(self):
    if len(self.memory) < self.minibatch_size:
      return
    minibatch = random.sample(self.memory, self.minibatch_size)
    for state, action, reward, next_state, done in minibatch:
      faux_batch = np.reshape(state, [1, self.state_size])
      target = self.model.predict(faux_batch)
      if done:
        target[0][action] = reward
      else:
        faux_batch = np.reshape(next_state, [1, self.state_size])
        t = self.target_model.predict(faux_batch)[0]
        target[0][action] = reward + self.gamma * np.amax(t)
      faux_batch = np.reshape(state, [1, self.state_size])
      self.model.fit(faux_batch, target, epochs=1, verbose=0)
    self.policy.post_train_action()

# vim: set ts=2 sw=2 expandtab:
