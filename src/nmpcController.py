#! /usr/bin/env python
'''
Explicação do funcionamento
'''

import numpy as np

from utils.inputClasses.controllerParams import ControllerParams
from utils.inputClasses.controllerInput import ControllerInput
from utils.functions.scaleForSaturation import scale_for_saturation

class NMPC_Controller():
    def __init__(self, input: ControllerInput) -> None:
        self.xref, self.yref, self.RstateX, self.RstateY, \
        self.RstateTheta, self.RstateVelocity, \
	    self.RstateW, self.xrefA, self.yrefA, \
        self.thetarefA, self.vrefA, self.wrefA = input

        # Inicialização dos parâmetros
        self.max_velocity, self.wheels_distance, self.predict_horz, \
        self.predict_horz_end, self.control_horz, self.gain_xy_error, \
        self.gain_theta_error, self.gain_delta_control, \
        self.eta, self.max_iterations, self.actual_iteration, self.delta = self.init_controller_params()

        _, _, _, self.prediction_model = self.init_predict_model()

    def __str__(self):
        return f'xref:{self.xref}\nyref: {self.yref}'

    def init_controller_params(self) -> ControllerParams:
        controller_params = ControllerParams(
            max_velocity = 0.4,
            wheels_distance = 0.23,
            predict_horz = 1,
            predict_horz_end = 10,
            control_horz = 2,
            gain_xy_error = 10,
            gain_theta_error = 2.5,
            gain_delta_control = 0.85,
            eta = 0.1,
            max_iterations = 15,
            actual_iteration = 0,
            delta = 0.1
        )

        return controller_params

    def init_optmizer(self, alpha):
        '''Parâmetros do Otimizador
        alpha: Passo do otimizador do gradiente descendente.

        Inicializa os vetores do Otimizador
        Jsteps: Vetor de passos J 8x1. Onde J é a função custo.
        Jgrad: Vetor de passos do Gradiente de J 4x1.
        Jgrad_prev: Vetor de passos do Gradiente Anterior de J 4x1.
        '''
        # alpha = alpha
        Jsteps = np.zeros((1, self.control_horz*4))
        Jgrad = np.zeros((1, self.control_horz*2))
        Jgrad_prev = np.zeros((1, self.control_horz*2))

        return alpha, Jsteps, Jgrad, Jgrad_prev


    def init_predict_model(self):
        '''Inicializa o modelo do preditor
        
        Retorna:
        Ubest: Melhor U control_horz = 2x2. 2 pq são V e W(ohmega) e porque são os dois horizontes de controle.
        Uref: Saída do controle de referência. U final JxNu = 2x2.
        Uaux: Saída do controle auxiliar que vai servir para o otimizador.
        '''
        Ubest = np.zeros(2,self.control_horz) 
        Uref = np.zeros(2, self.control_horz) 
        Uaux = np.zeros(2, self.control_horz)

        SinRob = {'x': 0, 'y': 0, 'theta': 0, 'v': 0, 'w': 0}

        return Ubest, Uref, Uaux, SinRob
    

    def update_prediction_model(self, TRsx, TRsy, TRst, RstateVelocity, RstateW):
        '''
        SinRob.x = TRsx;
        SinRob.y = TRsy;
        SinRob.theta = TRst;
        SinRob.v = RstateVelocity;
        SinRob.w = RstateW;
        '''
        self.prediction_model['x'] = TRsx
        self.prediction_model['y'] = TRsy
        self.prediction_model['theta'] = TRst
        self.prediction_model['v'] = RstateVelocity
        self.prediction_model['w'] = RstateW
    

    def speed_saturation(self, Uref, Vmax):
        new_output_ref = scale_for_saturation(Uref, self.wheels_distance, self.predict_horz, Vmax)
        return new_output_ref

    def calculate_cust_function():
