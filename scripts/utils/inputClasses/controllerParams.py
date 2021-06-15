from typing import NamedTuple

class ControllerParams(NamedTuple):
    """A configuration for the NMP controller.

    Attributes:
        max_velocity: maximu robot speeed.
        whells_distance: distância entre as rodas.
        
        predict_horz: Horizonte de predição (N1).
        predict_horz_end: final do horizonte de predição (N2).
        control_horz: Horizonte de controle (Nu).

        gain_xy_error: ganho para o erro x e y (L1).
        gain_theta_error: ganho para o erro em Theta (L2).
        gain_delta_control: ganho para variação de controle (L3).

        eta: critério de parada
        max_iterations: número máximo de iterações.
        actual_iteration: contador de iteração do Otimizador
        delta: passo do U (passo da iteração)
    """
    max_velocity: float
    whells_distance: float
    predict_horz: float
    predict_horz_end: float
    control_horz: float

    gain_xy_error: float
    gain_theta_error: float
    gain_delta_control: float

    eta: int
    max_iterations: int
    actual_iteration: int
    delta: float