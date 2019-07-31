"""Definition of the RewardCenter class.
"""

import numpy as np
from events import (
  CubeSpottedEvent,
  CubeCollectedEvent,
  CubeDroppedEvent,
  FutileCollectAttemptEvent
)

class RewardCenter(object):
  def __init__(
    self,
    swarmie_id_list,
    emitter,
    spotting_bonus=0.1,
    collect_bonus=1.0,
    bad_drop_penalty=-0.5,
    futile_collect_penalty=-0.1
  ):
    super(RewardCenter, self).__init__()
    self.spotting_bonus = spotting_bonus
    self.collect_bonus = collect_bonus
    self.bad_drop_penalty = bad_drop_penalty
    self.futile_collect_penalty = futile_collect_penalty
    self.swarmie_id_list = swarmie_id_list
    self.reward_map = self._new_reward_map(swarmie_id_list)
    emitter.on_event(CubeSpottedEvent, self.new_cube_spotted_by)
    emitter.on_event(CubeCollectedEvent, self.cube_collected_by)
    emitter.on_event(CubeDroppedEvent, self.cube_dropped_by)
    emitter.on_event(FutileCollectAttemptEvent, self.swarmie_attempted_futile_collect)

  def _new_reward_map(self, swarmie_id_list):
    return dict(zip(swarmie_id_list, [0.,] * len(swarmie_id_list)))
  
  def new_cube_spotted_by(self, event):
    self.reward_map[event.swarmie_id] += self.spotting_bonus
  
  def cube_collected_by(self, event):
    self.reward_map[event.swarmie_id] += self.collect_bonus
  
  def cube_dropped_by(self, event):
    self.reward_map[event.swarmie_id] += self.bad_drop_penalty

  def swarmie_attempted_futile_collect(self, event):
    self.reward_map[event.swarmie_id] += self.futile_collect_penalty

  def reward_for(self, swarmie_id):
    return self.reward_map[swarmie_id]

  def reset(self):
    self.reward_map = self._new_reward_map(self.swarmie_id_list)

# vim: set ts=2 sw=2 expandtab:
