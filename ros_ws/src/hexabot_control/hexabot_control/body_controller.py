import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hexabot_control.config import load_geometry
from hexabot_control.kinematics import solve_all_legs

from hexabot_interfaces.msg import LegJointTargets, TipTargets

class (Node):
     def __init__(self):
            super().__init__('leg_ik')