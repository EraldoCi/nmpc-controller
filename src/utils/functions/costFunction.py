#! /usr/bin/env python
# OLHAR NO TEMPO 53 MIN
import math

from .angleDif import angle_difference
from ..inputClasses.costFuncParams import CostFunctionParams

PI = math.pi

def cost_function (values: CostFunctionParams) -> float:
    velocities_vector = values.velocities_vector # SRv
    robot_theta = values.robot_theta # SRtheta
    predict_horz = values.predict_horz # N1 = 1
    control_horz = values.control_horz # Nu = 2
    predict_horz_end = values.predict_horz_end # N2 = 10
    x_position = values.x_position # SRx
    y_position = values.y_position # SRy
    x_ref_pos = values.x_ref_pos # Xref
    y_ref_pos = values.y_ref_pos # Yref
    linear_v = values.linear_v
    angular_v_w = values.angular_v_w

    sum_x = 0
    sum_y = 0
    sum_theta = 0
    work_time = 0.04 # tempo(s) de trabalho

	# Prediction horizon
    for i in range (0, predict_horz_end):
        if i <= control_horz: # Nu
                v = velocities_vector[1, i] # U(1, i)
                w = velocities_vector[2, i]
        else:
            v = velocities_vector[1, predict_horz]
            w = velocities_vector[2, predict_horz]
		
	# NÚMERO DE VEZES CORRESPONDENTE A FREQUÊNCIA DE TRABALHO
	# SE ESTOU TRABALHANDO A 40ms, DEVO CALCULAR 4 VEZES
        for _ in range (4):
            c_theta = math.cos(robot_theta) 
            s_theta = math.sin(robot_theta)

            if robot_theta > PI:
                robot_theta = robot_theta - 2*PI

            robot_theta = robot_theta + work_time*w
            robot_theta = x_position + work_time*(v*c_theta)
            robot_theta = y_position + work_time*(v*s_theta)
	
	# Valor da função custo para cada horizonte de predição
        sum_x += (x_ref_pos[i] - x_position)**2 
        sum_y += (y_ref_pos[i] - y_position)**2
        sum_theta += angle_difference(values.Theta_ref[i], robot_theta)**2
        # sum_theta = sum_theta + DifAngle(PHIref(i), SRTheta)^2;

    deltaVelocity = (linear_v - predict_horz[1,1])**2 + (angular_v_w - predict_horz[2,1])**2 # Delta U
    component1_cost = values.gain_xy_error * (sum_x + sum_y)
    component2_cost = values.gain_theta_error * sum_theta
    component3_cost = values.gain_delta_control * deltaVelocity

    J = component1_cost + component2_cost + component3_cost
    return J
