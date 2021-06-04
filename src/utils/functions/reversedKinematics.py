import math

def reversed_kinematic (vx, vy, ax, ay):
    '''
    vx: Robot linear velocity in X.
    vy: Robot linear velocity in Y.
    ax: Previous X robot position.
    ay: Previous Y robot position.

    '''
    theta = math.atan2(vy, vx)
    v = math.sqrt(vx**2 + vy**2)
    w = (vx*ay - vy*ax)/(vx**2 + vy**2)

    '''
    The v, w & theta must be the reference
    '''
    return v, w, theta