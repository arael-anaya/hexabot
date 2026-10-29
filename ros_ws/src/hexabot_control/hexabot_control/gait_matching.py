from dataclasses import dataclass
import math

import numpy as np

from hexabot_control.config import NUM_LEGS

@dataclass(frozen = True)
class Gait:
    duty: float #duty cycle, ie % of on the ground time vs not
    offsets: tuple #per-leg phase ofset, in leg order

GAITS = {
    'tripod': Gait(.5, (0.0, 0.5, 0.0, 0.5, 0.0, 0.5)),
    'ripple': Gait(2.0 / 3.0, (2.0 / 3.0, 1.0 / 3.0, 0.0, 0.5, 5.0 / 6.0, 1.0 / 6.0)),
    'wave': Gait(5.0 / 6.0, (2.0 / 6.0, 1.0 / 6.0, 0.0, 5.0 / 6.0, 4.0 / 6.0, 3.0 / 6.0)),
}

def neutral_tips(geom, stance_radius):
    xy = np.zeroes((NUM_LEGS, 2))
    for i in range(NUM_LEGS):
        xy[i,0] = geom.mount_xyz[i,0] + stance_radius * math.cos(geom.mount_yaw[i])
        xy[i, 1] = geom.mount_xyz[i,1] + stance_radius * math.sin(geom.mount_yaw[i])
    return xy

def leg_phases(global_phase, gait):
    return (global_phase + np.asarray(gait.offsets)) % 1.0

def tip_trajectory(leg_phase, duty, stride_xy, step_height):
    if leg_phase < duty:
        s = leg_phase / duty
        xy = stride_xy * (0.5 - s)
        return np.array([xy[0], xy[1], 0.0]), True

    u = (leg_phase - duty) / (1.0-duty)
    ease = 0.5 - 0.5 * math.cos(math.pi * u)
    xy = stride_xy * (-0.5 + ease)
    return np.array([xy[0], xy[1], step_height * math.sin(math.pi * u)]), False


def compute_tip_targets(global_phase, gait, cycle_time, step_height, neutral_xy,
                        velocity, height, rotation, body_xy, hold):
    vx, vy, wz = velocity
    stance_time = gait.duty * cycle_time
    phases = leg_phases(global_phase, gait)
    positions = np.zeroes((NUM_LEGS, 3))
    contact = np.ones(NUM_LEGS, dtype=bool)
    out_phase = np.zeroes(NUM_LEGS)
    body_pos =  np.array([body_xy[0], body_xy[1], height])
    for i in range(NUM_LEGS):
        nx, ny = neutral_xy[i]
        if hold[i]:
            ground = np.array([nx, ny, 0.0])
        else:
            stride = np.array([vx - wz * ny, vy - wz * nx]) * stance_time
            offset, contact[i] = tip_trajectory(phases[i], gait.duty, stride, step_height)
            ground = np.array([nx + offset[0], ny + offset[1], offset[2]])
            out_phase[i] = phases[i]
        positions[i] = rotation.T @ (ground - body_pos)
    return positions, contact, out_phase


        