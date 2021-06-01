import numpy as np
import math

from utils.functions import angleDif

PI = math.pi

'''
lTi: .
Rx: Robot X position state.
Ry: Robot Y position state.
Rt: Robot Theta state.
trajX: X previous position.
trajY: Y previous position.
trajTeta: Theta previous reference.
V: previous linear velocity reference.
W: previous angular velocity reference.
N2: prediction horizon end (?).
trajXp: X reference position.
trajYp: Y reference position.

[tPx, tPY, tPTheta] = calcRefTraj(lTi, Rsx, Rsy, Rst, xrefA, yrefA, tetarefA, ...
		vrefA, wrefA, N2, xref, yref);
'''

def calculate_mini_trajectory(lTi, SRx, SRy, SRt, trajX, trajY, trajTeta, V, W, N2, trajXp, trajYp):
    '''
    Calculates the Reference trajectory
    '''
    N = 2001
    i = 1
    deltaD = V*0.04 # 0.1 para Pioneer e 0.04 para Turtlebot
    deltaW = W*0.04
    dTotal = 0

    trajPX= np.zeros(1, N2+1)
    trajPY= np.zeros(1, N2+1)
    trajPTeta= np.zeros(1, N2+1)

    #angle of current segment to world
    teta = np.arctan2(trajYp-trajY, trajXp-trajX)

    #angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
    al = angleDif(trajTeta, SRt) - teta
    d1_3 = math.sqrt( (trajX-SRx)**2 + (trajY-SRy)**2 )
    
    #distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
    dl = d1_3 * math.cos(al)
    segmentSize = math.sqrt( (trajXp-trajX)**2 + (trajYp-trajY)**2)

    #sum of distances from segments (of main trajectory)
    dSegments = segmentSize - dl
        
    #initial point of predictive controller trajectory
    trajPX[i] = trajX + dl*math.cos(teta)
    trajPY[i] = trajY + dl*math.sin(teta)
    trajPTeta[i] = trajTeta

    #build new trajectory
    for j in range(2, N2): # j=2:1:N2+1
        #reached the end of the trajectory
        if lTi >= N-1 :
            trajPX[j] = trajX
            trajPY[j] = trajY
            trajPTeta[j] = trajTeta
        else:
            #add trajectory points along current segment
            trajPX[j] = trajPX[j-1] + deltaD*math.cos(teta)
            trajPY[j] = trajPY[j-1] + deltaD*math.sin(teta)
            trajPTeta[j] = trajPTeta[j-1] + deltaW
                
            dTotal = dTotal + deltaD
                
            #change segment of the main trajectory that's being tracked
            if dTotal >= dSegments :
                lTi = lTi + 1
                teta = np.arctan2(trajYp-trajY, trajXp-trajX)
                segmentSize = math.sqrt( (trajXp-trajX)**2 + (trajYp-trajY)**2 ) 

                #add point (already in next segment)
                trajPX[j] = trajX + (dTotal-dSegments)*math.cos(teta)
                trajPY[j] = trajY + (dTotal-dSegments)*math.sin(teta)
                trajPTeta[j] = trajTeta
                
                dSegments = dSegments + segmentSize

    #handle teta references
    for i in range(0, N2): # ab=1:1:N2
        if trajPTeta[i] > PI:
            trajPTeta[i] = trajPTeta[i] - 2*PI

    return trajPX, trajPY, trajPTeta
