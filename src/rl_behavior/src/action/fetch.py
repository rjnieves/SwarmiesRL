"""Definition of the FetchAction class.
"""

from . import MoveToCellAction, SweepAction, ApproachAction, PickupAction, DropOffAction

class FetchAction(object):
  MOVE_TO_TARGET_STATE = 1
  SWEEPING_SITE_STATE = 2
  APPROACHING_TARGET_STATE = 3
  PICKING_UP_TARGET_STATE = 4
  MOVE_TO_NEST_STATE = 5
  DROP_OFF_TARGET_STATE = 6

  def __init__(self, swarmie_name, arena, target_grid_coords, tag_state):
    super(FetchAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.arena = arena
    self.target_grid_coords = target_grid_coords
    self.tag_state = tag_state
    self._current_state = None
    self._current_action = None

  def _move_complete_event(self, swarmie_state, elapsed_time):
    tag_spotted = bool(self.tag_state.cube_tags)
    if self._current_state == FetchAction.MOVE_TO_TARGET_STATE:
      if tag_spotted:
        self._current_state = FetchAction.APPROACHING_TARGET_STATE
        self._current_action = ApproachAction(self.swarmie_name, self.tag_state)
        return self._current_action.update(swarmie_state, elapsed_time)
      else:
        self._current_state = FetchAction.SWEEPING_SITE_STATE
        self._current_action = SweepAction(self.swarmie_name, self.tag_state)
        return self._current_action.update(swarmie_state, elapsed_time)
    elif self._current_state == FetchAction.MOVE_TO_NEST_STATE:
      self._current_state = FetchAction.DROP_OFF_TARGET_STATE
      self._current_action = DropOffAction(self.swarmie_name)
      return self._current_action.update(swarmie_state, elapsed_time)
    else:
      return None

  def _sweep_complete_event(self, swarmie_state, elapsed_time):
    tag_spotted = bool(self.tag_state.cube_tags)
    if self._current_state == FetchAction.SWEEPING_SITE_STATE:
      if tag_spotted:
        self._current_state = FetchAction.APPROACHING_TARGET_STATE
        self._current_action = ApproachAction(self.swarmie_name, self.tag_state)
        return self._current_action.update(swarmie_state, elapsed_time)
      else:
        # TODO: Indicate failure to spot supposed target
        return None
    else:
      return None

  def _approach_complete_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.APPROACHING_TARGET_STATE:
      self._current_state = FetchAction.PICKING_UP_TARGET_STATE
      self._current_action = PickupAction(self.swarmie_name)
      return self._current_action.update(swarmie_state, elapsed_time)
    else:
      return None

  def _pickup_complete_event(self, swarmie_state, elapsed_time):
    if self._current_state == FetchAction.PICKING_UP_TARGET_STATE:
      # TODO: Determine if we actually picked up the target!
      self._current_state = FetchAction.MOVE_TO_NEST_STATE
      self._current_action = MoveToCellAction(self.swarmie_name, self.arena, self.arena.nest_grid_tl)
      return self._current_action.update(swarmie_state, elapsed_time)
    else:
      return None

  def _drop_off_complete_event(self, swarmie_state, elapsed_time):
    # TODO: Determine if we actually scored!
    return None

  def update(self, swarmie_state, elapsed_time):
    if self._current_state is None:
      self._current_state = FetchAction.MOVE_TO_TARGET_STATE
      self._current_action = MoveToCellAction(self.swarmie_name, self.arena, self.target_grid_coords)
    next_response = self._current_action.update(swarmie_state, elapsed_time)
    if next_response is None:
      if isinstance(self._current_action, MoveToCellAction):
        next_response = self._move_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, SweepAction):
        next_response = self._sweep_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, ApproachAction):
        next_response = self._approach_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, PickupAction):
        next_response = self._pickup_complete_event(swarmie_state, elapsed_time)
      elif isinstance(self._current_action, DropOffAction):
        next_response = self._drop_off_complete_event(swarmie_state, elapsed_time)
    return next_response

# vim: set ts=2 sw=2 expandtab:
