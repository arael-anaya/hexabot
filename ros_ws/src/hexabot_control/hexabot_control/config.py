"""Shared constants and placeholder robot geometry for the hexabot nodes."""

from dataclasses import dataclass
import math

import numpy as np

NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_JOINTS = NUM_LEGS * JOINTS_PER_LEG

MODE_SIT = 0
MODE_STAND = 1
MODE_WALK = 2
MODE_ESTOP = 3


DEFAULT_MOUNT_POSITIONS = [
    0.038027, 0.069125, 0.0,
    0.058250, 0.000000, 0.0,
    0.035891, -0.079191, 0.0,
    -0.035891, -0.079191, 0.0,
    -0.058250, 0.000000, 0.0,
    -0.038027, 0.069125, 0.0,
]
DEFAULT_MOUNT_YAW = [1.067866, 0.000000, -1.145274, -1.996319, 3.141593, 2.073727]

DEFAULT_JOINT_SIGN = [1.0, -1.0, -1.0] * 6

DEFAULT_JOINT_OFFSET = [
    0.229070, -0.189026, 0.114996,
    0.033156, 0.389557, -0.000000,
    -0.722166, 0.360634, 0.019830,
    0.107364, 0.027190, 0.012129,
    -0.071282, 0.045266, 0.334725,
    0.078608, -0.725019, 0.784777,
]
DEFAULT_LINK1 = 0.048  # joint 1 -> joint 2 (a*_joint2 origin x)
DEFAULT_LINK2 = 0.056  # joint 2 -> joint 3 (a*_joint3 origin x)
DEFAULT_LINK3 = 0.068  # joint 3 -> foot tip (link3 mesh extent, URDF has no foot link)


# Class for robot geometry
@dataclass(frozen=True)
class RobotGeometry:
    mount_xyz: np.ndarray  # (6, 3) joint 1 position in the body frame
    mount_yaw: np.ndarray  # (6,) direction each leg points, rad, about body z
    link1: float
    link2: float
    link3: float

# Names joints
def default_joint_names():
    names = []
    for leg in range(1, NUM_LEGS + 1):
        for joint in (1, 2, 3):
            names.append(f'a{leg}_joint{joint}')
    return names

# loads robot geometry
def load_geometry(node):
    """Declare and read the shared geometry parameters on a node."""
    positions = node.declare_parameter('leg_mount_positions', DEFAULT_MOUNT_POSITIONS).value
    yaws = node.declare_parameter('leg_mount_yaw', DEFAULT_MOUNT_YAW).value
    link1 = node.declare_parameter('link1_length', DEFAULT_LINK1).value
    link2 = node.declare_parameter('link2_length', DEFAULT_LINK2).value
    link3 = node.declare_parameter('link3_length', DEFAULT_LINK3).value
    if len(positions) != 3 * NUM_LEGS or len(yaws) != NUM_LEGS:
        raise ValueError('leg_mount_positions needs 18 values and leg_mount_yaw needs 6')
    return RobotGeometry(
        mount_xyz=np.array(positions, dtype=float).reshape(NUM_LEGS, 3),
        mount_yaw=np.array(yaws, dtype=float),
        link1=float(link1), link2=float(link2), link3=float(link3))
