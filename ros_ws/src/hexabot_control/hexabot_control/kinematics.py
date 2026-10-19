"""Hexapod leg kinematics (no ROS imports).

Leg frame: origin at joint 1, x pointing out along the leg's mount yaw, z up.
Each leg has three links; joint k drives link k. Joint angles q = (q1, q2, q3), radians:
  joint 1 - yaw about z, swings the whole leg
  joint 2 - pitch, positive lifts link 2 up
  joint 3 - pitch of link 3 relative to link 2; <= 0 folds link 3 down
The tip is the end of link 3.
"""

import math

import numpy as np

from hexabot_control.config import NUM_LEGS


def leg_fk(q, link1, link2, link3):
    q1, q2, q3 = q
    r = link1 + link2 * math.cos(q2) + link3 * math.cos(q2 + q3)
    z = link2 * math.sin(q2) + link3 * math.sin(q2 + q3)
    return np.array([r * math.cos(q1), r * math.sin(q1), z])


def leg_ik(tip, link1, link2, link3, margin=1e-3):
    """Return (q, reachable). Unreachable targets are clamped to the workspace edge."""
    x, y, z = tip
    q1 = math.atan2(y, x)
    r = math.hypot(x, y) - link1
    d = math.hypot(r, z)
    d_max = link2 + link3 - margin
    d_min = abs(link2 - link3) + margin
    reachable = True
    if d > d_max:
        r, z, d = r * d_max / d, z * d_max / d, d_max
        reachable = False
    elif d < d_min:
        if d < 1e-9:
            r, z = d_min, 0.0
        else:
            r, z = r * d_min / d, z * d_min / d
        d = d_min
        reachable = False
    cos_q3 = (d * d - link2 * link2 - link3 * link3) / (2.0 * link2 * link3)
    q3 = -math.acos(max(-1.0, min(1.0, cos_q3)))
    q2 = math.atan2(z, r) - math.atan2(link3 * math.sin(q3), link2 + link3 * math.cos(q3))
    return np.array([q1, q2, q3]), reachable


def body_to_leg(p_body, mount_xyz, mount_yaw):
    d = np.asarray(p_body, dtype=float) - mount_xyz
    c, s = math.cos(mount_yaw), math.sin(mount_yaw)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])


def leg_to_body(p_leg, mount_xyz, mount_yaw):
    c, s = math.cos(mount_yaw), math.sin(mount_yaw)
    return mount_xyz + np.array(
        [c * p_leg[0] - s * p_leg[1], s * p_leg[0] + c * p_leg[1], p_leg[2]])


def solve_all_legs(tips, geom):
    """tips: (6, 3) in the body frame. Returns q (18,) and reachable (6,) bool."""
    q = np.zeros(3 * NUM_LEGS)
    reachable = np.ones(NUM_LEGS, dtype=bool)
    for i in range(NUM_LEGS):
        p_leg = body_to_leg(tips[i], geom.mount_xyz[i], geom.mount_yaw[i])
        q[3 * i:3 * i + 3], reachable[i] = leg_ik(p_leg, geom.link1, geom.link2, geom.link3)
    return q, reachable
