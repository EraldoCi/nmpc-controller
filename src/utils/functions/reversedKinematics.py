import math

def reversed_kinematic (vx, vy, ax, ay):
    '''
    vx: Robot linear velocity in X.
    vy: Robot linear velocity in Y.
    ax: Previous X robot position.
    ay: Previous Y robot position.
    '''

    # print(f'''
    #     VX: {vx}
    #     W: {vy}
    #     THETA: {ax} {ay}
    # ''')

    theta = math.atan2(vy, vx)
    v = math.sqrt(vx**2 + vy**2)
    w = 0.0
    try:
        w = float((vx*ay - vy*ax)/(vx**2 + vy**2))
    except Exception as e:
        w = 0.0
        print(f'Exception is:{e}\n W VALUE IS: {w}')

    # if math.isnan:
    #     print(f'WWWWWWWWWWWWWWWW: {w}')
    #     w = 0.0

    # print(f'''
    #     reversed_kinematic V: {v}
    #     reversed_kinematic W: {w}
    #     reversed_kinematic THETA: {theta}
    # ''')

    '''
    The v, w & theta must be the reference
    '''
    return v, w, theta