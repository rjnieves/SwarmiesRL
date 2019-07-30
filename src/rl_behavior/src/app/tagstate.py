"""Definition of the TagState class.
"""

import rospy
import numpy as np
import tf
from scipy.spatial import distance
from tf.transformations import euler_from_quaternion

class TagState(object):
  class Report(object):
    MAXIMUM_AGE = rospy.Duration(1, 400000000)

    @classmethod
    def sort_key(cls, instance):
      return instance.tag_dist

    def __init__(self, world_coords, grid_coords, base_link_coords):
      super(TagState.Report, self).__init__()
      if len(world_coords) != 2:
        raise ValueError(
          'TagState.Report world_coords not a 1-D, 2-element tuple.'
        )
      if len(grid_coords) != 2:
        raise ValueError(
          'TagState.Report grid_coords not a 1-D, 2-element tuple.'
        )
      if len(base_link_coords) != 3:
        raise ValueError(
          'TagState.Report base_link_coords not a 1-D, 3-element tuple.'
        )
      world_coords = np.array(world_coords, dtype=np.float64)
      grid_coords = np.array(grid_coords, dtype=np.int16)
      base_link_coords = np.array(base_link_coords, dtype=np.float64)
      self.world_coords = tuple(world_coords)
      self.grid_coords = tuple(grid_coords)
      self.base_link_coords = tuple(base_link_coords)
      self.tag_dist = distance.euclidean((0., 0.), self.base_link_coords[0:2])
      self._age_countdown = TagState.Report.MAXIMUM_AGE
    
    def dock_age(self, by_amt):
      self._age_countdown -= min(self._age_countdown, by_amt)

    @property
    def alive(self):
      return bool(self._age_countdown)
    
    @property
    def is_new(self):
      return self._age_countdown == TagState.Report.MAXIMUM_AGE

    @property
    def age(self):
      return TagState.Report.MAXIMUM_AGE - self._age_countdown

    def __repr__(self):
      return (
        'TagState.Report(' +
        'world_coords={}, ' +
        'grid_coords={}, ' +
        'base_link_coords={})'
      ).format(
        repr(self.world_coords),
        repr(self.grid_coords),
        repr(self.base_link_coords)
      )

  def __init__(self, swarmie_name, swarmie_state, tf, coord_xform):
    super(TagState, self).__init__()
    self.swarmie_name = swarmie_name
    self.swarmie_state = swarmie_state
    self.coord_xform = coord_xform
    self.tf = tf
    self.reset()
  
  def reset(self):
    self.cube_tags = []
    self.boundary_tags = []
    self.nest_tags = []

  def sort(self):
    self.cube_tags.sort(key=TagState.Report.sort_key)
    self.boundary_tags.sort(key=TagState.Report.sort_key)
    self.nest_tags.sort(key=TagState.Report.sort_key)
    return self

  def dock_age(self, by_amt):
    for a_list in [self.cube_tags, self.boundary_tags, self.nest_tags]:
      for a_tag_report in a_list:
        a_tag_report.dock_age(by_amt)
    self.cube_tags = [a_tag for a_tag in self.cube_tags if a_tag.alive]
    self.boundary_tags = [a_tag for a_tag in self.boundary_tags if a_tag.alive]
    self.nest_tags = [a_tag for a_tag in self.nest_tags if a_tag.alive]
    return self

  def update(self, tag_list):
    for a_tag_detect in tag_list.detections:
      target_frame = ''
      resulting_pose = a_tag_detect.pose
      (_, _, yaw) = euler_from_quaternion(
        (
          resulting_pose.pose.orientation.x,
          resulting_pose.pose.orientation.y,
          resulting_pose.pose.orientation.z,
          resulting_pose.pose.orientation.w
        )
      )
      base_link_coords = None
      tag_best_guess = None
      try:
        for target_id in ['base_link', 'odom',]:
          target_frame = '{swarmie}/{target}'.format(
            swarmie=self.swarmie_name,
            target=target_id
          )
          self.tf.waitForTransform(
            target_frame,
            resulting_pose.header.frame_id,
            resulting_pose.header.stamp,
            rospy.Duration(nsecs=100000000)
          )
          resulting_pose = self.tf.transformPose(
            target_frame,
            resulting_pose
          )
          if target_id == 'base_link':
            (_, _, yaw) = euler_from_quaternion(
              (
                resulting_pose.pose.orientation.x,
                resulting_pose.pose.orientation.y,
                resulting_pose.pose.orientation.z,
                resulting_pose.pose.orientation.w
              )
            )
            base_link_coords = (
              resulting_pose.pose.position.x,
              resulting_pose.pose.position.y,
              yaw
            )
        tag_best_guess = self.swarmie_state.local_odom_to_global(
          (
            round(resulting_pose.pose.position.x, 1),
            round(resulting_pose.pose.position.y, 1),
            0.
          )
        )
        tag_grid_coord = self.coord_xform.from_real_to_grid(
          tag_best_guess[0:2]
        )
        tag_report = TagState.Report(
          world_coords=tag_best_guess[0:2],
          grid_coords=tag_grid_coord,
          base_link_coords=base_link_coords
        )
        if a_tag_detect.id == 0:
          self._merge_report(tag_report, self.cube_tags)
        elif a_tag_detect.id == 128:
          self._merge_report(tag_report, self.boundary_tags)
        elif a_tag_detect.id == 255:
          self._merge_report(tag_report, self.nest_tags)
      except tf.Exception:
        rospy.logwarn(
          '{} cannot transform from {} to {}'.format(
            self.swarmie_name,
            resulting_pose.header.frame_id,
            target_frame
          )
        )
    return self

  def _merge_report(self, tag_report, tag_list):
    tag_idx = None
    for idx, a_tag in enumerate(tag_list):
      if a_tag.world_coords == tag_report.world_coords:
        tag_idx = idx
        break
    if tag_idx is not None:
      del tag_list[tag_idx]
    tag_list.append(tag_report)

# vim: set ts=2 sw=2 expandtab:
