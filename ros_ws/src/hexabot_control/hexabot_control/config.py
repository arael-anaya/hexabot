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
    0.282468, 0.000000, 0.031817,
    0.000000, 0.000000, 0.031817,
    -0.359875, 0.000000, 0.031817,
    0.273164, 0.093948, -0.168169,
    0.000983, 0.000000, 0.031817,
    -0.282468, 0.000000, 0.031817,
]
DEFAULT_LINK1 = 0.032515  # joint 1 -> joint 2 (|a*_joint2 origin xy|)
DEFAULT_LINK2 = 0.058000  # joint 2 -> joint 3 (|a*_joint3 origin xy|)
DEFAULT_LINK3 = 0.073265  # joint 3 -> foot tip (link3 mesh tip, URDF has no foot link)


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
