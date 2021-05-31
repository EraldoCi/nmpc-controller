import numpy as np

# Retorna dU[] vetor 2x8
def calculate_Usteps(velocities_vector, control_horz, delta):
    delta_vel_vector = np.zeros(2, control_horz*4)

    for i in range(0, len(control_horz)):
        delta_vel_vector[1, 1+4*i] = velocities_vector[1, i+1] + delta
        delta_vel_vector[1, 1+4*i] = velocities_vector[2, i+1]

        delta_vel_vector[1, 2+4*i] = velocities_vector[1, i+1] - delta
        delta_vel_vector[2, 2+4*i] = velocities_vector[2, i+1]

        delta_vel_vector[1, 3+4*i] = velocities_vector[1, i+1]
        delta_vel_vector[2, 3+4*i] = velocities_vector[2, i+1] + delta

        delta_vel_vector[1, 4+4*i] = velocities_vector[1, i+1]
        delta_vel_vector[2, 4+4*i] = velocities_vector[2, i+1] - delta

    return delta_vel_vector
 