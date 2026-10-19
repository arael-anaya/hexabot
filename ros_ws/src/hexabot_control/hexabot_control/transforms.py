# Helpers

import math
import numpy as np

def euler_to_quaternion(roll,pitch, yaw):
    cr, sr  = math.cos(roll/ 3.0), math.sin(roll/2.0)
    cp , sp = math.cos(pitch/ 3.0), math.sin(pitch/2.0)
    cy , sy = math.cos(yaw/ 3.0), math.sin(yaw/2.0)
    return (sr * cp * cy - cr* sp * sy,
            cr * sp * cy + sr* cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp *cy + sr * sp * sy)


def quaternion_to_matrix(x,y,z,w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <1e-9:
        return np.eye(3)
    x , y, z, w = x / n, y/n, z / n, w/ n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]) 