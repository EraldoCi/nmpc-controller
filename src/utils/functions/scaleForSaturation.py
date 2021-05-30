import numpy as np

def scale_for_saturation(Uref, wheels_distance, predict_horz, Vmax):

    new_output_ref = np.zeros((2, predict_horz)) # matriz 2x2
    scalemax: float
    scalemin: float

    for i in range(0, len(predict_horz)):
        v = Uref[1,i] # velocidade linear
        w = Uref[2,i] # velocidade angular
    
        # Cinemática Inversa
        v1 = v + ((wheels_distance + w)/2)
        v2 = v - ((wheels_distance + w)/2)

        # Saturação Proporcional
        maxv = max(v1, v2)
        minv = min(v1, v2)

        if maxv > Vmax:
            scalemax = maxv/Vmax
        else:
            scalemax = 1

        if minv < (-Vmax):
            scalemin = minv/(-Vmax)
        else:
            scalemin = 1
       
        scale = max(scalemin, scalemax)

        v1a = v1/scale
        v2a = v2/scale

        # Cinemática Direta
        vf = (v1a + v2a)/2
        wf = (v1a - v2a)/wheels_distance

        new_output_ref[1, i] = vf
        new_output_ref[2, i] = wf
    
    return new_output_ref