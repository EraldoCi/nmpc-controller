import numpy as np
import math

from .angleDif import angle_difference
from utils.inputClasses import TrajectoryInput

PI = math.pi

'''
Rx: Robot X position state.
Ry: Robot Y position state.
Rt: Robot Theta state.
trajX: X previous position.
trajY: Y previous position.
trajTeta: Theta previous reference.
V: previous linear velocity reference.
W: previous angular velocity reference.
N2: prediction horizon end (10).
trajXp: X reference position.
trajYp: Y reference position.

[tPx, tPY, tPTheta] = calcRefTraj(lTi, Rsx, Rsy, Rst, xrefA, yrefA, tetarefA, ...
		vrefA, wrefA, N2, xref, yref);
'''

def calculate_mini_trajectory( input_params: TrajectoryInput):
    SRx, SRy, SRt, trajX, trajY, trajTeta, V, W, N2, trajXp, trajYp = input_params
    
    # print(trajXp, trajYp)

    iteration = 0
    N = 2001
    deltaD = V*0.4 # 0.1 para Pioneer e 0.04 para Turtlebot
    deltaW = W*0.4
    dTotal = 0

    trajPX = np.zeros(N2)
    trajPY = np.zeros(N2)
    trajPTeta = np.zeros(N2)

    #angle of current segment to world
    teta = np.arctan2(trajYp-trajY, trajXp-trajX)

    #angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
    al = angle_difference(trajTeta, SRt) - teta
    d1_3 = math.sqrt( (trajX-SRx)**2 + (trajY-SRy)**2 )
    
    #distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
    dl = d1_3 * math.cos(al)
    segmentSize = math.sqrt( (trajXp-trajX)**2 + (trajYp-trajY)**2)

    #sum of distances from segments (of main trajectory)
    dSegments = segmentSize - dl
        
    #initial point of predictive controller trajectory
    trajPX[0] = trajX + dl*math.cos(teta)
    trajPY[0] = trajY + dl*math.sin(teta)
    trajPTeta[0] = trajTeta

    #build new trajectory
    for j in range(1, N2): # j=2:1:N2+1
        #reached the end of the trajectory
        if iteration >= N-1 :
            trajPX[j] = trajX
            trajPY[j] = trajY
            trajPTeta[j] = trajTeta
        else:
            #add trajectory points along current segment
            trajPX[j] = trajPX[j-1] + deltaD*math.cos(teta)
            trajPY[j] = trajPY[j-1] + deltaD*math.sin(teta)
            trajPTeta[j] = trajPTeta[j-1] + deltaW
                
            dTotal = dTotal + deltaD
            trajPTeta[j] = math.atan2(trajPY[j], trajPX[j])

            #change segment of the main trajectory that's being tracked
            if dTotal >= dSegments :
                iteration += 1
                teta = np.arctan2(trajYp-trajY, trajXp-trajX)
                segmentSize = math.sqrt( (trajXp-trajX)**2 + (trajYp-trajY)**2 ) 

                #add point (already in next segment)
                trajPX[j] = trajX + (dTotal-dSegments)*math.cos(teta)
                trajPY[j] = trajY + (dTotal-dSegments)*math.sin(teta)
                trajPTeta[j] = trajTeta
                
                # calculates 
                trajPTeta[j] = math.atan2(trajPY[j], trajPX[j])


                dSegments = dSegments + segmentSize

    #handle teta references
    for i in range(0, N2): # ab=1:1:N2
        if trajPTeta[i] > PI:
            trajPTeta[i] = trajPTeta[i] - 2*PI

    # print(f'X TRAJECTORY: {trajPX}\nY TRAJECTORY: {trajPY}')

    return trajPX, trajPY, trajPTeta
