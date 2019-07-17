from __future__ import print_function
import math
import rospy
import message_filters
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Joy

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
  WARMUP_TIME = rospy.Duration(30, 0)
  TIMESTEP_PERIOD = rospy.Duration(0, 100000000)
  JOYSTICK_LINEAR_AXIS = 4
  JOYSTICK_ANGUAR_AXIS = 3

  def __init__(self, swarmie_name):
    self.swarmie_name = swarmie_name
    self.swarmie_id = RlBehavior.NAME_TO_ID_MAP.index(swarmie_name)
    self.center_location = {
      'map': None,
      'odom': None
    }
    self.current_location = {
      'map': None,
      'odom': None
    }
    # Publishers ---------------------------------------------------------------
    self.status_pub = None
    self.hb_pub = None
    self.infolog_pub = None
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
  
  def run(self):
    print('Welcome to the world of tomorrow {}!'.format(self.swarmie_name))
    rospy.init_node('{}_BEHAVIOUR'.format(self.swarmie_name))
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
    return (
      self.center_location['map'] is not None and
      self.center_location['odom'] is not None
    )

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
      # First time initialization
      self.infolog_pub.publish(
        String(
          data='{} ready to learn!'.format(self.swarmie_name)
        )
      )
      self.center_location['odom'] = Pose2D(
        x=1.3 * math.cos(self.current_location['odom'].theta),
        y=1.3 * math.sin(self.current_location['odom'].theta)
      )
      rospy.loginfo(
        '{} odom center at ({},{}), current yaw ({})'.format(
          self.swarmie_name,
          self.center_location['odom'].x,
          self.center_location['odom'].y,
          self.current_location['odom'].theta
        )
      )
      self.center_location['map'] = Pose2D(
        x=(
          self.current_location['map'].x +
          (1.3 * math.cos(self.current_location['map'].theta))
        ),
        y=(
          self.current_location['map'].y +
          (1.3 * math.sin(self.current_location['map'].theta))
        )
      )
      rospy.loginfo(
        '{} map center at({},{}), current map yaw ({})'.format(
          self.swarmie_name,
          self.center_location['map'].x,
          self.center_location['map'].y,
          self.current_location['map'].theta
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
    rospy.logdebug(
      '{} local position: ({},{})'.format(
        self.swarmie_name,
        sample.pose.pose.position.x,
        sample.pose.pose.position.y
      )
    )
    (_, _, yaw) = euler_from_quaternion(
      (
        sample.pose.pose.orientation.x,
        sample.pose.pose.orientation.y,
        sample.pose.pose.orientation.z,
        sample.pose.pose.orientation.w
      )
    )
    self.current_location['odom'] = Pose2D(
      x=sample.pose.pose.position.x,
      y=sample.pose.pose.position.y,
      theta=yaw
    )

  def _on_map_update(self, sample):
    """Map-based position update callback.

    Called by ROS whenever a sample arrives via the <swarmie>/odom/ekf topic,
    communicating the currently-sensed position of the swarmie using a global
    reference frame.

    :param sample: Odometry sample.
    :type sample: nav_msgs.msg.Odometry
    """
    rospy.logdebug(
      '{} map position: ({},{})'.format(
        self.swarmie_name,
        sample.pose.pose.position.x,
        sample.pose.pose.position.y
      )
    )
    (_, _, yaw) = euler_from_quaternion(
      (
        sample.pose.pose.orientation.x,
        sample.pose.pose.orientation.y,
        sample.pose.pose.orientation.z,
        sample.pose.pose.orientation.w
      )
    )
    self.current_location['map'] = Pose2D(
      x=sample.pose.pose.position.x,
      y=sample.pose.pose.position.y,
      theta=yaw
    )

  def _on_mode_change(self, sample):
    """Operating mode update callback.

    Called by ROS whenever a sample arrives via the <swarmie>/mode topic,
    indicating either a mode initialization or a mode change.

    :param sample: UInt8 sample containing the new mode.
    :type sample: std_msgs.msg.UInt8
    """
    if sample.data == RlBehavior.AUTONOMOUS_MODE:
      rospy.loginfo(
        '{} switching to AUTONOMOUS_MODE'.format(self.swarmie_name)
      )
      self.infolog_pub.publish(
        String(
          data='{} operating autonomously'.format(self.swarmie_name)
        )
      )
    elif sample.data == RlBehavior.MANUAL_MODE:
      rospy.loginfo(
        '{} switching to MANUAL_MODE'.format(self.swarmie_name)
      )
      self.infolog_pub.publish(
        String(
          data='{} under manual control'.format(self.swarmie_name)
        )
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
    rospy.loginfo(
      '{} joystick linear command speed={}'.format(
        self.swarmie_name,
        cmd.axes[RlBehavior.JOYSTICK_LINEAR_AXIS]
      )
    )
    rospy.loginfo(
      '{} joystick angular command speed={}'.format(
        self.swarmie_name,
        cmd.axes[RlBehavior.JOYSTICK_ANGUAR_AXIS]
      )
    )

# vim: set ts=2 sw=2 expandtab:
