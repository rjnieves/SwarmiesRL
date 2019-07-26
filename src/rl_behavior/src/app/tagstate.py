"""Definition of the TagState class.
"""

import rospy
import numpy as np
import tf
from scipy.spatial import distance
from tf.transformations import euler_from_quaternion

class TagState(object):
  class Report(object):
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

  def __init__(self, swarmie_name, location_info, tf, coord_xform):
    super(TagState, self).__init__()
    self.swarmie_name = swarmie_name
    self.location_info = location_info
    self.coord_xform = coord_xform
    self.tf = tf
    self._reset()
  
  def _reset(self):
    self.cube_tags = []
    self.boundary_tags = []
    self.nest_tags = []

  def filter(self):
    seen_tags = set()
    filtered_tags = []
    for a_cube_tag in self.cube_tags:
      if not a_cube_tag.world_coords in seen_tags:
        seen_tags.add(a_cube_tag.world_coords)
        filtered_tags.append(a_cube_tag)
    self.cube_tags = filtered_tags
    return self

  def sort(self):
    self.cube_tags.sort(key=TagState.Report.sort_key)
    self.boundary_tags.sort(key=TagState.Report.sort_key)
    self.nest_tags.sort(key=TagState.Report.sort_key)
    return self

  def update(self, tag_list):
    self._reset()
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
          (_, _, yaw) = euler_from_quaternion(
            (
              resulting_pose.pose.orientation.x,
              resulting_pose.pose.orientation.y,
              resulting_pose.pose.orientation.z,
              resulting_pose.pose.orientation.w
            )
          )
          if target_id == 'base_link':
            base_link_coords = (
              resulting_pose.pose.position.x,
              resulting_pose.pose.position.y,
              yaw
            )
        tag_best_guess = self.location_info.local_odom_to_global(
          (
            round(resulting_pose.pose.position.x, 1),
            round(resulting_pose.pose.position.y, 1),
            round(yaw, 2)
          )
        )
        rospy.logdebug('AprilTag at real coords {}'.format(tag_best_guess))
        tag_grid_coord = self.coord_xform.from_real_to_grid(
          tag_best_guess[0:2]
        )
        rospy.logdebug('Identifying AprilTag at {}'.format(tag_grid_coord))
        tag_report = TagState.Report(
          tag_best_guess[0:2],
          tag_grid_coord,
          base_link_coords
        )
        if a_tag_detect.id == 0:
          self.cube_tags.append(tag_report)
        elif a_tag_detect.id == 128:
          self.boundary_tags.append(tag_report)
        elif a_tag_detect.id == 255:
          self.nest_tags.append(tag_report)
      except tf.Exception:
        rospy.logwarn(
          '{} cannot transform from {} to {}'.format(
            self.swarmie_name,
            resulting_pose.header.frame_id,
            target_frame
          )
        )
    return self

# vim: set ts=2 sw=2 expandtab:
