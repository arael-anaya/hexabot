

from builtin_interfaces.msg import Duration
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hexabot_control.config import (
    MODE_SIT, DEFAULT_JOINT_OFFSET, DEFAULT_JOINT_SIGN, NUM_JOINTS, NUM_LEGS, default_joint_names)
from hexabot_control.limits import slew_array
from hexabot_interfaces.msg import LegJointTargets

# Hardware/URDF-frame limits; keep in sync with hexabot.urdf.
# Hip limits are centered on each leg's neutral offset (DEFAULT_JOINT_OFFSET) +/- HIP_RANGE.
HIP_RANGE = 0.4
DEFAULT_JOINT_MIN = []
DEFAULT_JOINT_MAX = []
for _i in range(NUM_LEGS):
    _hip = DEFAULT_JOINT_OFFSET[3 * _i]
    DEFAULT_JOINT_MIN += [round(_hip - HIP_RANGE, 3), -1.0, 0.162]
    DEFAULT_JOINT_MAX += [round(_hip + HIP_RANGE, 3), 1.0, 2.0]
# Boot pose: joint 3 folded to its lower limit. Held until a non-SIT mode_cmd arrives.
DEFAULT_BOOT_POSE = [0.0, 0.0, 0.162] * 6

class JointCommandBridge(Node):

    def __init__(self):
        super().__init__('joint_command_bridge')
        d = self.declare_parameter
        self.backend = d('backend', 'isaac').value #swaps between isaac or real world
        self.names = list(d('joint_names', default_joint_names()).value)
        self.sign = np.array(d('joint_sign', DEFAULT_JOINT_SIGN).value, dtype=float)
        self.offset = np.array(d('joint_offset', DEFAULT_JOINT_OFFSET).value, dtype=float)
        self.jmin = np.array(d('joint_min', DEFAULT_JOINT_MIN).value, dtype=float)
        self.jmax = np.array(d('joint_max', DEFAULT_JOINT_MAX).value, dtype=float)
        self.boot_pose = np.array(d('boot_pose', DEFAULT_BOOT_POSE).value, dtype=float)
        self.max_vel = d('max_joint_vel', 6.0).value
        self.watchdog = d('watchdog_timeout' , 0.2).value
        self.startup_wait = d('startup_wait' , 1.0).value
        self.dt = 1.0 / d('publish_rate', 100.0).value
        command_topic = d('command_topic' , '/isaac/joint_command').value
        trajectory_topic = d(
            'trajectory_topic', '/joint_trajectory_controller/joint_trajectory').value

        for arr in (self.names, self.sign, self.offset, self.jmin, self.jmax, self.boot_pose):
            if len(arr) != NUM_JOINTS:
                raise ValueError('joint_* parameters must all have 18 entries')
        if self.backend not in ('isaac', 'ros2_control'):
            raise ValueError('backend must be "isaac" or "ros2_control"')

        self.target = None
        self.last_rx = None
        self.first_rx = None
        self.prev = None
        self.actual = {}
        self.hold_boot = True
        self.t0 = self.get_clock().now()

        if self.backend == 'isaac':
            self.pub = self.create_publisher(JointState, command_topic, 10)
        else:
            self.pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)

        self.create_subscription(
            LegJointTargets, 'leg_joint_targets', self._on_targets, qos_profile_sensor_data)

        self.create_subscription(UInt8, 'mode_cmd', self._on_mode, 10)

        self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, qos_profile_sensor_data)
    
        self.create_timer(self.dt, self._tick)

    def _on_targets(self, msg):
        target = np.array(msg.positions, dtype=float)
        if target.shape != (NUM_JOINTS,) or not np.all(np.isfinite(target)):
            self.get_logger().warn('Bad leg_joint_targets dropped', throttle_duration_sec=2.0)
            return

        self.target = target
        self.last_rx = self.get_clock().now()
        if self.first_rx is None:
            self.first_rx = self.last_rx

    def _on_mode(self, msg):
        # Stay at the boot pose while in SIT; any other mode releases the legs.
        self.hold_boot = msg.data == MODE_SIT

    def _on_joint_states(self,msg):
        for name, pos in zip(msg.name, msg.position):
            self.actual[name] = pos

    def _to_hardware(self, model):
        return np.clip(self.sign* model + self.offset, self.jmin, self.jmax)

    def _initial_pose(self, now):
        if all (n in self.actual for n in self.names):
            return np.array([self.actual[n] for n in self.names])
        if (now - self.t0).nanoseconds * 1e-9 > self.startup_wait:
            self.get_logger().warn('No joint_states received, starting from the boot pose')
            return np.clip(self.boot_pose, self.jmin, self.jmax)
        return None

    def _tick(self):
        now = self.get_clock().now()
        if self.target is None and not self.hold_boot:
            return
        if self.prev is None:
            self.prev = self._initial_pose(now)
            if self.prev is None:
                return
        if self.hold_boot:
            goal = np.clip(self.boot_pose, self.jmin, self.jmax)
            self.prev = slew_array(self.prev, goal, self.max_vel * self.dt)
        elif (now - self.last_rx).nanoseconds * 1e-9 > self.watchdog:
            self.get_logger().warn(
                'No leg_joint_targets, holding last pose', throttle_duration_sec = 2.0)
        else:
            goal = self._to_hardware(self.target)
            self.prev = slew_array(self.prev, goal, self.max_vel * self.dt)
        self._publish(now)


    def _publish(self, now):
        if self.backend == 'isaac':
            msg = JointState()
            msg.header.stamp = now.to_msg()
            msg.name = self.names
            msg.position = self.prev.tolist()
        else:
            point = JointTrajectoryPoint()
            point.positions = self.prev.tolist()
            point.time_from_start = Duration(sec=0, nanosec=int(2e9 * self. dt))
            msg =  JointTrajectory()
            msg.joint_names = self.names
            msg.points = [point]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()  
    


