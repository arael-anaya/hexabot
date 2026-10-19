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

# Leg order, viewed from above, clockwise from the front-right leg:
#   1 front-right, 2 mid-right, 3 rear-right, 4 rear-left, 5 mid-left, 6 front-left
# Body frame: +x forward, +y left, +z up.
# TODO: every value below is a PLACEHOLDER. Replace with the real dimensions from CAD.
DEFAULT_MOUNT_POSITIONS = [
    0.10, -0.06, 0.0,
    0.00, -0.08, 0.0,
    -0.10, -0.06, 0.0,
    -0.10, 0.06, 0.0,
    0.00, 0.08, 0.0,
    0.10, 0.06, 0.0,
]
DEFAULT_MOUNT_YAW = [math.radians(a) for a in (-45.0, -90.0, -135.0, 135.0, 90.0, 45.0)]
DEFAULT_LINK1 = 0.05
DEFAULT_LINK2 = 0.10
DEFAULT_LINK3 = 0.15


@dataclass(frozen=True)
class RobotGeometry:
    mount_xyz: np.ndarray  # (6, 3) joint 1 position in the body frame
    mount_yaw: np.ndarray  # (6,) direction each leg points, rad, about body z
    link1: float
    link2: float
    link3: float


def default_joint_names():
    names = []
    for leg in range(1, NUM_LEGS + 1):
        for joint in (1, 2, 3):
            names.append(f'leg{leg}_joint{joint}')
    return names


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
