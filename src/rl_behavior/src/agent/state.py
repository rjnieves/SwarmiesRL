"""Definition of the StateRepository class.
"""

import math
import numpy as np
from scipy.spatial import distance
import rospy
from std_msgs.msg import Int16
from swarmie_msgs.msg import GridReport
from events import (
  SwarmieLocEvent,
  CubeSpottedEvent,
  CubePickedUpEvent,
  CubeCollectedEvent,
  CubeDroppedEvent,
  NewCubeEvent,
  CubeVanishedEvent
)

class StateRepository(object):
  def __init__(self, swarmie_id_list, own_id, cube_count, arena_dims, emitter):
    self.cube_count = cube_count
    self.arena_dims = arena_dims
    self.episode_reset(swarmie_id_list)
    self.own_swarmie_id = own_id
    self.emitter = emitter
    self.emitter.on_event(SwarmieLocEvent, self.local_swarmie_loc_update)
    self.emitter.on_event(CubeSpottedEvent, self.local_cube_spotted)
    self.emitter.on_event(CubePickedUpEvent, self.local_cube_picked_up_by)
    self.emitter.on_event(CubeCollectedEvent, self.local_cube_collected)
    self.emitter.on_event(CubeDroppedEvent, self.local_cube_dropped_by)
    self.emitter.on_event(CubeVanishedEvent, self.local_cube_vanished)
    self.cube_report_sub = None
    self.cube_report_pub = None
    self.pos_report_sub = None
    self.pos_report_pub = None
    self.cube_collection_sub = None
    self.cube_collection_pub = None
    self.cube_dropped_sub = None
    self.cube_dropped_pub = None
    self.cube_pickup_sub = None
    self.cube_pickup_pub = None
    self.cube_vanishing_sub = None
    self.cube_vanishing_pub = None

  @property
  def state_size(self):
    return len(self.cube_counts) + (len(self.swarmie_pos) * 2)

  def init_remote(self):
    # --------------------------------------------------------------------------
    # Set up all the publishers
    self.cube_report_pub = rospy.Publisher(
      '/cubeSpottedReport',
      GridReport,
      queue_size=10
    )
    self.pos_report_pub = rospy.Publisher(
      '/positionReport',
      GridReport,
      queue_size=10
    )
    self.cube_collection_pub = rospy.Publisher(
      '/collectionReport',
      Int16,
      queue_size=10
    )
    self.cube_dropped_pub = rospy.Publisher(
      '/dropReport',
      Int16,
      queue_size=10
    )
    self.cube_pickup_pub = rospy.Publisher(
      '/pickupReport',
      GridReport,
      queue_size=10
    )
    self.cube_vanishing_pub = rospy.Publisher(
      '/cubeVanishingReport',
      GridReport,
      queue_size=10
    )
    # --------------------------------------------------------------------------
    # Set up all the subscribers
    self.cube_report_sub = rospy.Subscriber(
      '/cubeSpottedReport',
      GridReport,
      callback=self.remote_cube_spotted,
      queue_size=10
    )
    self.pos_report_sub = rospy.Subscriber(
      '/positionReport',
      GridReport,
      callback=self.remote_swarmie_loc_update,
      queue_size=10
    )
    self.cube_collection_sub = rospy.Subscriber(
      '/collectionReport',
      Int16,
      callback=self.remote_cube_collected,
      queue_size=10
    )
    self.cube_dropped_sub = rospy.Subscriber(
      '/dropReport',
      Int16,
      callback=self.remote_cube_dropped_by,
      queue_size=10
    )
    self.cube_pickup_sub = rospy.Subscriber(
      '/pickupReport',
      GridReport,
      callback=self.remote_cube_picked_up_by,
      queue_size=10
    )
    self.cube_vanishing_sub = rospy.Subscriber(
      '/cubeVanishingReport',
      GridReport,
      callback=self.remote_cube_vanished,
      queue_size=10
    )

  def episode_reset(self, swarmie_id_list):
    self.spotted_cubes = set()
    self.swarmie_state = dict(
      zip(
        swarmie_id_list,
        [False,] * len(swarmie_id_list)
      )
    )
    self.cube_counts = {
      'at-large': self.cube_count,
      'located': 0,
      'in-transit': 0,
      'collected': 0
    }
    self.swarmie_pos = dict(
      zip(
        swarmie_id_list,
        [None, ] * len(swarmie_id_list)
      )
    )
    self.dist_meas = dict(
      zip(
        swarmie_id_list,
        [-1., ] * len(swarmie_id_list)
      )
    )
    self.nearest_cube = dict(
      zip(
        swarmie_id_list,
        [None, ] * len(swarmie_id_list)
      )
    )

  def _reeval_block_counts(self):
    at_large_total = self.cube_count
    self.cube_counts['in-transit'] = self.swarmie_state.values().count(True)
    at_large_total -= self.cube_counts['in-transit']
    self.cube_counts['located'] = len(self.spotted_cubes)
    at_large_total -= self.cube_counts['located']
    at_large_total -= self.cube_counts['collected']
    self.cube_counts['at-large'] = at_large_total

  def _reeval_distances(self):
    for a_swarmie_id in self.swarmie_pos.keys():
      closest = float('+inf')
      cube = None
      swarmie_pos = self.swarmie_pos[a_swarmie_id]
      if swarmie_pos is not None:
        for a_cube_loc in self.spotted_cubes:
          new_dist = distance.cityblock(
            np.array(a_cube_loc, dtype=int),
            np.array(swarmie_pos, dtype=int)
          )
          if new_dist < closest:
            closest = new_dist
            cube = a_cube_loc
      self.dist_meas[a_swarmie_id] = closest if not math.isinf(closest) else -1.
      self.nearest_cube[a_swarmie_id] = cube

  def make_state_vector(self):
    result = np.zeros(self.state_size)
    self._reeval_block_counts()
    self._reeval_distances()
    idx = 0
    for a_cube_cat in ['at-large', 'located', 'in-transit', 'collected', ]:
      result[idx] = float(self.cube_counts[a_cube_cat]) / float(self.cube_count)
      idx += 1
    for a_swarmie_id in sorted(self.swarmie_state.keys()):
      result[idx] = 1. if self.swarmie_state[a_swarmie_id] else 0.
      idx += 1
    for a_swarmie_id in sorted(self.dist_meas.keys()):
      result[idx] = float(self.dist_meas[a_swarmie_id])
      result[idx] /= float(np.sum(self.arena_dims))
      idx += 1
    return result

  def local_swarmie_loc_update(self, event):
    self.swarmie_pos[event.swarmie_id] = event.swarmie_loc
    if (
      self.pos_report_pub is not None and
      event.swarmie_id == self.own_swarmie_id
    ):
      self.pos_report_pub.publish(
        GridReport(
          swarmie_id=self.own_swarmie_id,
          grid_x=event.swarmie_loc[0],
          grid_y=event.swarmie_loc[1]
        )
      )
  
  def remote_swarmie_loc_update(self, report):
    if report.swarmie_id != self.own_swarmie_id:
      self.local_swarmie_loc_update(
        SwarmieLocEvent(
          swarmie_id=report.swarmie_id,
          swarmie_loc=(report.grid_x, report.grid_y)
        )
      )

  def local_cube_spotted(self, event):
    if event.swarmie_id == self.own_swarmie_id:
      if event.cube_loc not in self.spotted_cubes:
        self.emitter.emit(NewCubeEvent(event.cube_loc, event.swarmie_id))
      if self.cube_report_pub is not None:
        self.cube_report_pub.publish(
          GridReport(
            swarmie_id=self.own_swarmie_id,
            grid_x=event.cube_loc[0],
            grid_y=event.cube_loc[1]
          )
        )
    self.spotted_cubes.add(event.cube_loc)

  def remote_cube_spotted(self, report):
    if report.swarmie_id != self.own_swarmie_id:
      self.local_cube_spotted(
        CubeSpottedEvent(
          swarmie_id=report.swarmie_id,
          cube_loc=(report.grid_x, report.grid_y)
        )
      )

  def local_cube_picked_up_by(self, event):
    self.spotted_cubes.remove(event.cube_loc)
    self.swarmie_state[event.swarmie_id] = True
    if self.cube_pickup_pub is not None and event.swarmie_id == self.own_swarmie_id:
      self.cube_pickup_pub.publish(
        GridReport(
          swarmie_id=self.own_swarmie_id,
          grid_x=event.cube_loc[0],
          grid_y=event.cube_loc[1]
        )
      )

  def remote_cube_picked_up_by(self, report):
    if report.swarmie_id != self.own_swarmie_id:
      self.local_cube_picked_up_by(
        CubePickedUpEvent(
          swarmie_id=report.swarmie_id,
          cube_loc=(report.grid_x, report.grid_y)
        )
      )

  def local_cube_dropped_by(self, event):
    self.swarmie_state[event.swarmie_id] = False
    if self.cube_dropped_pub is not None and event.swarmie_id == self.own_swarmie_id:
      self.cube_dropped_pub.publish(
        Int16(data=self.own_swarmie_id)
      )

  def remote_cube_dropped_by(self, report):
    if report.data != self.own_swarmie_id:
      self.local_cube_dropped_by(
        CubeDroppedEvent(
          swarmie_id=report.data
        )
      )

  def local_cube_collected(self, event):
    self.swarmie_state[event.swarmie_id] = False
    self.cube_counts['collected'] += 1
    if self.cube_collection_pub is not None and event.swarmie_id == self.own_swarmie_id:
      self.cube_collection_pub.publish(
        Int16(data=self.own_swarmie_id)
      )

  def remote_cube_collected(self, report):
    if report.data != self.own_swarmie_id:
      self.local_cube_collected(
        CubeCollectedEvent(
          swarmie_id=report.data
        )
      )

  def local_cube_vanished(self, event):
    if event.swarmie_id == self.own_swarmie_id and self.cube_vanishing_pub is not None:
      self.cube_vanishing_pub.publish(
        GridReport(
          swarmie_id=self.own_swarmie_id,
          grid_x=event.cube_loc[0],
          grid_y=event.cube_loc[1]
        )
      )
    self.spotted_cubes.discard(event.cube_loc)

  def remote_cube_vanished(self, report):
    if report.swarmie_id != self.own_swarmie_id:
      self.local_cube_vanished(
        CubeVanishedEvent(
          swarmie_id=report.swarmie_id,
          cube_loc=(report.grid_x, report.grid_y)
        )
      )

  def swarmie_is_carrying(self, swarmie_id):
    return self.swarmie_state[swarmie_id]

# vim: set ts=2 sw=2 expandtab:
