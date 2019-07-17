"""Definition of the RewardCenter class.
"""

import numpy as np
from events import NewCubeEvent, CubeCollectedEvent

class RewardCenter(object):
  def __init__(
    self,
    swarmie_id_list,
    emitter,
    spotting_bonus=0.1,
    collect_bonus=1.0
  ):
    super(RewardCenter, self).__init__()
    self.spotting_bonus = spotting_bonus
    self.collect_bonus = collect_bonus
    self.swarmie_id_list = swarmie_id_list
    self.reward_map = self._new_reward_map(swarmie_id_list)
    emitter.on_event(NewCubeEvent, self.new_cube_spotted_by)
    emitter.on_event(CubeCollectedEvent, self.cube_collected_by)

  def _new_reward_map(self, swarmie_id_list):
    return dict(zip(swarmie_id_list, [0.,] * len(swarmie_id_list)))
  
  def new_cube_spotted_by(self, event):
    self.reward_map[event.swarmie_id] += self.spotting_bonus
  
  def cube_collected_by(self, event):
    self.reward_map[event.swarmie_id] += self.collect_bonus
  
  def reward_for(self, swarmie_id):
    return self.reward_map[swarmie_id]

  def reset(self):
    self.reward_map = self._new_reward_map(self.swarmie_id_list)

# vim: set ts=2 sw=2 expandtab:
