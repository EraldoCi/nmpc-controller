from typing import NamedTuple

class CostFunctionParams(NamedTuple):
    """Entrada da Função custo.

    Attributes:
        x_position: posição x do robô 
        y_position: posição y do robô 
        robot_theta: Theta do robô 
        linear_v: velocidade linear do robô
        angular_v_w: velocidade angular do robô
        
        velocities_vector: vetor de velocidades.
        
        predict_horz: Horizonte de predição (N1).
        predict_horz_end: final do horizonte de predição (Nw).
        control_horz: Horizonte de controle (Nu). 
    
        gain_xy_error: ganho para o erro x e y (L1). 
        gain_theta_error: ganho para o erro em Theta (L2).
        gain_delta_control: ganho para variação de controle (L2).
    """
    x_position: float 
    y_position: float 
    robot_theta: float 
    linear_v: float
    angular_v_w: float

    velocities_vector: list

    Xref: float 
    Yref: float
    PHIref: float 

    predict_horz: float
    predict_horz_end: float
    control_horz: float 

    gain_xy_error: float 
    gain_theta_error: float
    gain_delta_control: float