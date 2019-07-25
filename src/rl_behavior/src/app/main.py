from __future__ import print_function
import math
import numpy as np
import rospy
import message_filters
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Joy
from apriltags_ros.msg import AprilTagDetectionArray
from swarmie_msgs.msg import Skid, CubeReport
from world import CoordinateTransform
from . import LocationInformation

class RlBehavior(object):
  NAME_TO_ID_MAP = [
    'achilles',
    'aeneas',
    'ajax'
  ]
  HB_PERIOD = rospy.Duration(2, 0)
  STATUS_PERIOD = rospy.Duration(1, 0)
  MANUAL_MODE = 1
  AUTONOMOUS_MODE = 2
  MODE_DICT = {
    MANUAL_MODE: {
      'log_fmt': '{swarmie_name} switching to MANUAL_MODE',
      'infolog_fmt': '{swarmie_name} under manual control',
    },
    AUTONOMOUS_MODE: {
      'log_fmt': '{swarmie_name} switching to AUTONOMOUS_MODE',
      'infolog_fmt': '{swarmie_name} operating autonomously',
    }
  }
  WARMUP_TIME = rospy.Duration(30, 0)
  TIMESTEP_PERIOD = rospy.Duration(0, 100000000)
  JOYSTICK_LINEAR_AXIS = 4
  JOYSTICK_ANGULAR_AXIS = 3
  MAX_MOTOR_CMD = 255.0
  INIT_PLACE_RADIUS = 1.3
  ARENA_X_RANGE = (-7.5, 7.5)
  ARENA_Y_RANGE = (-7.5, 7.5)
  GRID_QUANTIZATION = (30, 30)
  NEST_X_TOP_LEFT = (-2.0, 2.0)
  NEST_DIMS = (4.0, 4.0)

  def __init__(self, swarmie_name):
    self.swarmie_name = swarmie_name
    self.swarmie_id = RlBehavior.NAME_TO_ID_MAP.index(swarmie_name)
    self.location_info = LocationInformation()
    self.mode = RlBehavior.MANUAL_MODE
    self.tf = None
    self.coord_xform = CoordinateTransform(
      RlBehavior.ARENA_X_RANGE,
      RlBehavior.ARENA_Y_RANGE,
      RlBehavior.GRID_QUANTIZATION
    )
    # Publishers ---------------------------------------------------------------
    self.status_pub = None
    self.hb_pub = None
    self.infolog_pub = None
    self.drive_cmd_pub = None
    self.cube_report_pub = None
    # Timers -------------------------------------------------------------------
    self.status_timer = None
    self.timestep_timer = None
    self.hb_timer = None
    # Subscribers --------------------------------------------------------------
    self.map_sub = None
    self.odom_sub = None
    self.mode_sub = None
    self.sonar_left_sub = None
    self.sonar_center_sub = None
    self.sonar_right_sub = None
    self.sonar_sync = None
    self.joy_sub = None
    self.target_sub = None
  
  def run(self):
    print('Welcome to the world of tomorrow {}!'.format(self.swarmie_name))
    rospy.init_node('{}_BEHAVIOUR'.format(self.swarmie_name))
    self.tf = TransformListener()
    # --------------------------------------------------------------------------
    # Set up all the publishers
    #
    self.status_pub = rospy.Publisher(
      '{}/swarmie_status'.format(self.swarmie_name),
      String,
      queue_size=10
    )
    self.hb_pub = rospy.Publisher(
      '{}/behaviour/heartbeat'.format(self.swarmie_name),
      String,
      queue_size=1
    )
    self.infolog_pub = rospy.Publisher(
      '/infoLog',
      String,
      queue_size=1
    )
    self.drive_cmd_pub = rospy.Publisher(
      '{}/driveControl'.format(self.swarmie_name),
      Skid,
      queue_size=10
    )
    self.cube_report_pub = rospy.Publisher(
      '/aprilCubeReports',
      CubeReport,
      queue_size=10
    )
    # --------------------------------------------------------------------------
    # Set up all the subscribers
    #
    self.map_sub = rospy.Subscriber(
      '{}/odom/ekf'.format(self.swarmie_name),
      Odometry,
      callback=self._on_map_update,
      queue_size=10
    )
    self.odom_sub = rospy.Subscriber(
      '{}/odom/filtered'.format(self.swarmie_name),
      Odometry,
      callback=self._on_odom_update,
      queue_size=10
    )
    self.mode_sub = rospy.Subscriber(
      '{}/mode'.format(self.swarmie_name),
      UInt8,
      callback=self._on_mode_change,
      queue_size=10
    )
    self.sonar_left_sub = message_filters.Subscriber(
      '{}/sonarLeft'.format(self.swarmie_name),
      Range
    )
    self.sonar_center_sub = message_filters.Subscriber(
      '{}/sonarCenter'.format(self.swarmie_name),
      Range
    )
    self.sonar_right_sub = message_filters.Subscriber(
      '{}/sonarRight'.format(self.swarmie_name),
      Range
    )
    self.sonar_sync = message_filters.TimeSynchronizer(
      [self.sonar_left_sub, self.sonar_center_sub, self.sonar_right_sub,],
      queue_size=10
    )
    self.sonar_sync.registerCallback(self._on_sonar_update)
    self.joy_sub = rospy.Subscriber(
      '{}/joystick'.format(self.swarmie_name),
      Joy,
      callback=self._on_joy_cmd,
      queue_size=10
    )
    self.target_sub = rospy.Subscriber(
      '/{}/targets'.format(self.swarmie_name),
      AprilTagDetectionArray,
      callback=self._on_tags_spotted,
      queue_size=10
    )
    # --------------------------------------------------------------------------
    # Set up all the timers
    #
    self.hb_timer = rospy.Timer(
      RlBehavior.HB_PERIOD,
      self._on_hb_timer
    )
    self.status_timer = rospy.Timer(
      RlBehavior.STATUS_PERIOD,
      self._on_status_timer
    )
    # --------------------------------------------------------------------------
    # Away we go!
    #
    rospy.Timer(RlBehavior.WARMUP_TIME, self._on_warmup_timer)
    rospy.spin()
  
  @property
  def initialized(self):
    return self.location_info.ready

  def _on_hb_timer(self, event):
    hb_msg = String()
    hb_msg.data = ''
    self.hb_pub.publish(hb_msg)
  
  def _on_status_timer(self, event):
    status_msg = String()
    status_msg.data = 'learners'
    self.status_pub.publish(status_msg)

  def _on_warmup_timer(self, event):
    # Start the timestep evaluation
    if self.timestep_timer is None:
      self.timestep_timer = rospy.Timer(
        RlBehavior.TIMESTEP_PERIOD,
        self._on_timestep_timer
      )

  def _on_timestep_timer(self, event):
    if not self.initialized:
      self.infolog_pub.publish(
        String(
          data='{} willing to learn!'.format(self.swarmie_name)
        )
      )
      self.location_info.initialize_centers(RlBehavior.INIT_PLACE_RADIUS)
      rospy.loginfo(
        '{} perceived center in odom frame at ({},{})'.format(
          self.swarmie_name,
          self.location_info.odom_center[0],
          self.location_info.odom_center[1]
        )
      )

  def _on_odom_update(self, sample):
    """Position update callback.

    Called by ROS whenever a sample arrives via the <swarmie>/odom/filtered
    topic, communicating the currently-sensed position of the swarmie using a
    local (starting point-centered) reference frame.

    :param sample: Sample containing position information.
    :type sample: nav_msgs.msg.Odometry
    """
    (_, _, yaw) = euler_from_quaternion(
      (
        sample.pose.pose.orientation.x,
        sample.pose.pose.orientation.y,
        sample.pose.pose.orientation.z,
        sample.pose.pose.orientation.w
      )
    )
    self.location_info.odom_current = np.array(
      [
        sample.pose.pose.position.x,
        sample.pose.pose.position.y,
        yaw
      ]
    )

  def _on_map_update(self, sample):
    """Map-based position update callback.

    Called by ROS whenever a sample arrives via the <swarmie>/odom/ekf topic,
    communicating the currently-sensed position of the swarmie using a global
    reference frame.

    :param sample: Odometry sample.
    :type sample: nav_msgs.msg.Odometry
    """
    (_, _, yaw) = euler_from_quaternion(
      (
        sample.pose.pose.orientation.x,
        sample.pose.pose.orientation.y,
        sample.pose.pose.orientation.z,
        sample.pose.pose.orientation.w
      )
    )
    self.location_info.map_current = np.array(
      [
        sample.pose.pose.position.x,
        sample.pose.pose.position.y,
        yaw
      ]
    )

  def _on_mode_change(self, sample):
    """Operating mode update callback.

    Called by ROS whenever a sample arrives via the <swarmie>/mode topic,
    indicating either a mode initialization or a mode change.

    :param sample: UInt8 sample containing the new mode.
    :type sample: std_msgs.msg.UInt8
    """
    if sample.data != self.mode:
      if sample.data in RlBehavior.MODE_DICT:
        self.mode = sample.data
        dict_entry = RlBehavior.MODE_DICT[sample.data]
        rospy.loginfo(
          dict_entry['log_fmt'].format(swarmie_name=self.swarmie_name)
        )
        self.infolog_pub.publish(
          dict_entry['infolog_fmt'].format(swarmie_name=self.swarmie_name)
        )
      else:
        rospy.logwarn(
          '{} cannot switch to unknown mode {}'.format(
            self.swarmie_name,
            sample.data
          )
        )

  def _on_sonar_update(self, left_sample, center_sample, right_sample):
    """Sonar data update callback.

    Called by ROS when it is able to deliver a time-homogeneous collection of
    samples from the three (3) sonar sensors in the swarmies.

    :param left_sample: Sample from the left range finder.
    :type left_sample: sensor_msgs.msg.Range
    :param center_sample: Sample from the center range finder.
    :type center_sample: sensor_msgs.msg.Range
    :param right_sample: Sample from the right range finder.
    :type right_sample: sensor_msgs.msg.Range
    """
    rospy.logdebug(
      '{} sonar readings: L:{} C:{} R:{}'.format(
        self.swarmie_name,
        left_sample.range,
        center_sample.range,
        right_sample.range
      )
    )

  def _on_joy_cmd(self, cmd):
    """Joystick command callback.

    Called by ROS whenever a Joystick command is emitted in the system.

    :param cmd: Joystick command sample.
    :type cmd: sensor_msgs.msg.Joy
    """
    if self.mode == RlBehavior.MANUAL_MODE:
      # ------------------------------------------------------------------------
      # Scale linear motor command, enforcing a +/- 0.1 "dead zone"
      linear_thresh = math.fabs(
        cmd.axes[RlBehavior.JOYSTICK_LINEAR_AXIS]
      ) >= 0.1
      linear_cmd = (
        cmd.axes[RlBehavior.JOYSTICK_LINEAR_AXIS] *
        RlBehavior.MAX_MOTOR_CMD
      )
      linear_cmd = 0.0 if not linear_thresh else linear_cmd
      # ------------------------------------------------------------------------
      # Scale angular motor command, enforcing a +/- 0.1 "dead zone"
      angular_thresh = math.fabs(
        cmd.axes[RlBehavior.JOYSTICK_ANGULAR_AXIS]
      ) >= 0.1
      angular_cmd = (
        cmd.axes[RlBehavior.JOYSTICK_ANGULAR_AXIS] *
        RlBehavior.MAX_MOTOR_CMD
      )
      angular_cmd = 0.0 if not angular_thresh else angular_cmd
      # ------------------------------------------------------------------------
      # Calculate left robot tracks command, restricting the command value to
      # the range [-MAX_MOTOR_CMD, MAX_MOTOR_CMD]
      left_cmd = linear_cmd - angular_cmd
      left_cmd = min(left_cmd, RlBehavior.MAX_MOTOR_CMD)
      left_cmd = max(left_cmd, -1.0 * RlBehavior.MAX_MOTOR_CMD)
      # ------------------------------------------------------------------------
      # Calculate right robot tracks command, restricting the command value to
      # the range [-MAX_MOTOR_CMD, MAX_MOTOR_CMD]
      right_cmd = linear_cmd + angular_cmd
      right_cmd = min(right_cmd, RlBehavior.MAX_MOTOR_CMD)
      right_cmd = max(right_cmd, -1.0 * RlBehavior.MAX_MOTOR_CMD)
      self.drive_cmd_pub.publish(
        Skid(left=left_cmd, right=right_cmd)
      )

  def _on_tags_spotted(self, tag_list):
    """April tag detection callback.

    Called by ROS whenever the April tag CV task spots tags in images returned
    by the swarmie camera.

    :param tag_list: List of April tags spotted by CV algorithm.
    :type tag_list: apriltags_ros.msg.AprilTagDetectionArray
    """
    if not self.initialized:
      return
    for a_tag_detect in tag_list.detections:
      if a_tag_detect.id == 0:
        target_frame = ''
        resulting_pose = a_tag_detect.pose
        cube_best_guess = None
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
          cube_best_guess = self.location_info.local_odom_to_global(
            (
              resulting_pose.pose.position.x,
              resulting_pose.pose.position.y,
              yaw
            )
          )
          rospy.loginfo('AprilCube at real coords {}'.format(cube_best_guess))
          cube_grid_coord = self.coord_xform.from_real_to_grid(
            cube_best_guess[0:2]
          )
          rospy.loginfo('Reporting AprilCube at {}'.format(cube_grid_coord))
          self.cube_report_pub.publish(
            CubeReport(
              grid_x = cube_grid_coord[0],
              grid_y = cube_grid_coord[1],
              swarmie_id = self.swarmie_id
            )
          )
        except tf.Exception:
          rospy.logwarn(
            '{} cannot transform from {} to {}'.format(
              self.swarmie_name,
              resulting_pose.header.frame_id,
              target_frame
            )
          )

# vim: set ts=2 sw=2 expandtab:
