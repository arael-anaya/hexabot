import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8

from hexabot_control.config import (
    MODE_ESTOP, MODE_SIT, MODE_STAND, MODE_WALK, NUM_LEGS, load_geometry)
from hexabot_control.gait_matching import GAITS, compute_tip_targets, neutral_tips
from hexabot_control.limits import clamp, slew
from hexabot_interfaces.msg import TipTargets
from hexabot_interfaces.srv import SetGait, SetMode

class (Node):
     def __init__(self):
            super().__init__('leg_ik')
            d = self.declare_parameter
            self.geom = load_geometry(self)
            