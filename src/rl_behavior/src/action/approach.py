"""Definition of the ApproachAction class.
"""

import math
import rospy
from swarmie_msgs.msg import Skid
from app import PidLoop

class ApproachAction(object):
  CUBE_TARGET_DISTANCE = 0.25 # meters
  DIST_TO_VEL_KP = 0.2 # kP for simple distance to velocity P-loop
  MAX_APPROACH_VEL = 0.2 # meters/s
  MIN_APPROACH_VEL = 0.1 # meters/s
  MIN_SKID_CMD = -180.
  MAX_SKID_CMD = 180.

  def __init__(self, swarmie_name, tag_state):
    super(ApproachAction, self).__init__()
    self.swarmie_name = swarmie_name
    self.tag_state = tag_state
    self.vel_pid = None
    self.yaw_pid = None
  
  def update(self, **kwargs):
    linear_vel = float(kwargs.get('linear_vel', 0.))
    if not self.tag_state.cube_tags:
      rospy.loginfo(
        '{} asked to approach non-existent cube.'.format(
          self.swarmie_name
        )
      )
      self.approaching = False
      self.vel_pid = None
      self.yaw_pid = None
      return None

    closest_cube = self.tag_state.cube_tags[0]
    rospy.loginfo(
      '{} approaching cube at grid {}, base_link {}, {} meters off.'.format(
        self.swarmie_name,
        closest_cube.grid_coords,
        closest_cube.base_link_coords,
        closest_cube.tag_dist
      )
    )
    if closest_cube.tag_dist > ApproachAction.CUBE_TARGET_DISTANCE:
      if self.vel_pid is None:
        self.vel_pid = PidLoop(PidLoop.Config.make_slow_vel())
      if self.yaw_pid is None:
        self.yaw_pid = PidLoop(PidLoop.Config.make_slow_yaw())
      vel_setpoint = closest_cube.tag_dist * ApproachAction.DIST_TO_VEL_KP
      vel_setpoint = min(ApproachAction.MAX_APPROACH_VEL, vel_setpoint)
      vel_setpoint = max(ApproachAction.MIN_APPROACH_VEL, vel_setpoint)
      vel_error = vel_setpoint - linear_vel
      rospy.loginfo(
        '{} velocity set point is ({}), current is ({}), making an error of ({})'.format(
          self.swarmie_name,
          vel_setpoint,
          linear_vel,
          vel_error
        )
      )
      yaw_setpoint = 0.
      yaw_current = math.atan(
        closest_cube.base_link_coords[1] / closest_cube.base_link_coords[0]
      )
      yaw_error = yaw_setpoint - yaw_current
      rospy.loginfo(
        '{} yaw set point is ({}), current is ({}), making an error of ({})'.format(
          self.swarmie_name,
          yaw_setpoint,
          yaw_current,
          yaw_error
        )
      )
      vel_output = self.vel_pid.pid_out(vel_error, vel_setpoint)
      # Negating the sign of the error since transverse y-axis positive
      # direction is to the left.
      yaw_output = self.yaw_pid.pid_out(-1. * yaw_error, yaw_setpoint)
      rospy.loginfo(
        '{} PID output for velocity ({}), yaw ({})'.format(
          self.swarmie_name,
          vel_output,
          yaw_output
        )
      )
      left_cmd = vel_output - yaw_output
      left_cmd = min(ApproachAction.MAX_SKID_CMD, left_cmd)
      left_cmd = max(ApproachAction.MIN_SKID_CMD, left_cmd)
      right_cmd = vel_output + yaw_output
      right_cmd = min(ApproachAction.MAX_SKID_CMD, right_cmd)
      right_cmd = max(ApproachAction.MIN_SKID_CMD, right_cmd)
      return Skid(left=left_cmd, right=right_cmd)
    else:
      rospy.loginfo(
        '{} done with approach to cube.'.format(
          self.swarmie_name
        )
      )
      self.vel_pid = None
      self.yaw_pid = None
      return None

# vim: set ts=2 sw=2 expandtab: