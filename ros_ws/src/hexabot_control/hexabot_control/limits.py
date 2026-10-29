import numpy as np

def clamp(value, low, high):
    return max(low, min(high, value))

def slew(current, target, max_step):
    return current + clamp(target - current, -max_step, max_step)

def slew_array(current, target, max_step):
    return current + np.clip(target - current, -max_step,max_step)

