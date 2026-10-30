

from builtin_interface.msg import Duration
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.quos import qos_profile_sensor_data
from sensor.msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hexabot_control.config import NUM_JOINTS, default_joint_names
from hexabot_control.limits import slew_array
from hexabot_interfaces.msg import LegJointTargets

DEFAULT_JOINT_MIN = [-.9, -1.5, -2.6] * 6
DEFAULT_JOINT_MAX = [.9, 1.5, 0.05] * 6

class JointCommandBridge(Node):

    def __init__(self):
        super().__init__('joint_command_bridge')
        d = self.declare_parameter
        self.backend = d('backend', 'issac').value #seaps between issac or real world
        self.names = list(d('joint_names', default_joint_names()).value)
        self.sign = np.array(d('joint_sign', [1.0] * NUM_JOINTS).value, dtype=float)
        self.offset = np.array(d('joint_offset', [0.0] * NUM_JOINTS).value, dtype=float)
        self.min = np.array(d('joint_min', DEFAULT_JOINT_MIN).value, dtype=float)
        self.max = np.array(d('joint_max', DEFAULT_JOINT_MIN).value, dtype=float)
        self.max_vel = d('max_joint_vel', 6.0).value
        self.watchdow = d('watchdog_timeout' , 0.2).value
        self.startup_wait = d('statup_wait' , 1.0).value
        self.dt = 1.0 / d('publish_rate', 100.0).value
        command_topic = d('command_topic' , '/issac/joint_command').value
        trajectory_topic = d(
            'trajectory_topic', '/joint_trajectory_controller/joint_trajectory').value

        for arr in (self.names, self.sign, self.offset, self.jmin, self.jmax):
            if len(arr) != NUM_JOINTS:
                raise ValueError('joint _* parameters must all have 18 entries')
            if self.backend not in ('issac', 'ros2_control'):
                raise ValueError('backend must be "issac" or "ros2_control"')

        self.target = None
        self.laxt_rx = None
        self.first_rx = None
        self.prev = None
        self.actual = {}

        if self.backend == 'issac':
            self.pub = self.create_publisher(JointState, trajectory_topic, 10)
        else:
            self.pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)

        self.create_subscription(
            LegJointTargets, 'leg_joint_targets', self._on_targets, qos_profile_sensor_data)

        self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, qos_profile_sensor_data)
    
        self.create_timer(self.dt, self._tick)

    def _on_targets(self, msg):
        target = np.array(msg.positions, dtype=float)
        if target.shape != (NUM_JOINTS,) or not np.all(np.isfinite(target)):
            self.get_logger().warn('Bad leg_joint_targets dropped', throttle_duration_sec=2.0)
            return

        self.target = target
        self.laxt_rx = self.get_clock().now()
        if self.first_rx is None:
            self.first_rx = self.laxt_rx
        
    


