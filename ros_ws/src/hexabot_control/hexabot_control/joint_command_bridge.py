

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
    




