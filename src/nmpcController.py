#! /usr/bin/env python
'''
Explicação do funcionamento
'''

V_MAX = 0.4 # Vmax
WHEELS_DISTANCE = 0.23 # d
PREDICT_HORZ = 1 # N1
PREDICT_HORZ_END = 10 # n2
CONTROL_HORZ = 2 # Nu
GAIN_XY_ERROR = 10 # L1
GAIN_THETA_ERROR = 2.5 # L2
GAIN_DELTA_ERROR = 0.85 # L3
ETA = 0.1
MAX_ITERATIONS = 15 
DELTA = 0.1

import numpy as np

from utils.inputClasses import ControllerParams, ControllerInput, CostFunctionParams
from utils.functions import calculate_Usteps, calculate_mini_trajectory, cost_function, scale_for_saturation

class NMPC_Controller():
    def __init__(self, input: ControllerInput) -> None:
        self.xref, \
        self.yref, \
        self.RstateX, \
        self.RstateY, \
        self.RstateTheta, \
        self.RstateVelocity, \
	    self.RstateW, \
        self.xrefA, \
        self.yrefA, \
        self.thetarefA, \
        self.vrefA, \
        self.wrefA = input

        # Inicialização dos parâmetros
        self.max_velocity, \
        self.wheels_distance, \
        self.predict_horz, \
        self.predict_horz_end, \
        self.control_horz, \
        self.gain_xy_error, \
        self.gain_theta_error, \
        self.gain_delta_control, \
        self.eta, \
        self.max_iterations, \
        self.actual_iteration, \
        self.delta = self.init_controller_params()

        self.Ubest, self.velocity_reference, self.Uaux, self.prediction_model = self.init_predict_model()
        #   Uref
        self.actual_cost
        self.best_cost

    def __str__(self):# used for debuging
        return f'xref:{self.xref}\nyref: {self.yref}'

    def init_controller_params(self) -> ControllerParams:
        controller_params = ControllerParams(
            max_velocity = 0.4,
            wheels_distance = 0.23,
            predict_horz = 1,
            predict_horz_end = 10,
            control_horz = 2, # Nu
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
        Jsteps: Vetor de passos J 1x8. Onde J é a função custo.
        Jgrad: Vetor de passos do Gradiente de J 4x1.
        Jgrad_prev: Vetor de passos do Gradiente Anterior de J 4x1.
        '''
        # alpha = alpha
        Jsteps = np.zeros((1, self.control_horz*4))
        Jgrad = np.zeros((1, self.control_horz*2))
        Jgrad_prev = np.zeros((1, self.control_horz*2))

        return alpha, Jsteps, Jgrad, Jgrad_prev


    def init_predict_model(self):
        '''Inicializa o modelo do preditor.
        
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
    

    def init_controller_variables(self, Uref):
        '''
        Um dos componentes para inicializar as variáveis
        do controlador.
        '''
        for i in range(0, len(self.predict_horz)):
            Uref[1, i] = self.vrefA
            Uref[2, i] = self.wrefA
        
        return Uref


    def calculate_trajactory(self):
        '''
        Olhar no minuto 40
        * Ver o que é feito nesta função
        '''
        calculate_mini_trajectory()
        return


    def update_prediction_model(self, Rsx, Rsy, Rst, Rsv, Rsw):
        ''' Atualização do modelo de predição.
        Rsx: robot X state 
        Rsy: robot Y state
        Rst: robot Theta state
        Rsv: robot linear velocity state
        Rsw: robot angular velocity state
        '''
        self.prediction_model['x'] = Rsx
        self.prediction_model['y'] = Rsy
        self.prediction_model['theta'] = Rst
        self.prediction_model['v'] = Rsv
        self.prediction_model['w'] = Rsw
    

    def speed_saturation(self, Uref):
        '''
            Satura a velocidade.
        '''
        new_output_ref = scale_for_saturation(Uref, self.wheels_distance, self.predict_horz, self.max_velocity)
        return new_output_ref


    def calculate_cust_function(self) -> float:
        # lembrar de alterar os campos: 
        #  x_ref_pos, y_ref_pos, Theta_ref
        cost_input = CostFunctionParams(
            x_position = self.prediction_model['x'], 
            y_position = self.prediction_model['y'], 
            robot_theta = self.prediction_model['theta'] , 
            linear_v = self.prediction_model['v'],
            angular_v_w = self.prediction_model['w'],
            velocities_vector = self.velocity_reference, # Uref
            x_ref_pos = self.xrefA, # tPX
            y_ref_pos = self.yrefA, # tPY
            Theta_ref = self.thetarefA, # yPTheta
            predict_horz = self.predict_horz,
            predict_horz_end = self.predict_horz_end,
            control_horz = self.control_horz, 
            gain_xy_error = self.gain_xy_error, 
            gain_theta_error = self.gain_theta_error ,
            gain_delta_control = self.gain_delta_control,
        )
        
        return cost_function(cost_input) # self.actual_cost = cost_function(cost_input) 


    def start_optmizer(self):
        
        # Cálculo da trajatória de referência (mini trajetoria)
        lTi = 0
        # [tPx, tPY, tPTheta] = calcRefTraj(lTi, TRsx, TRsy, TRst, Xref, Yref, PHlref, ...
        #     Vref, Wref, N2, Xrefp, Yrefp);
        tPx, tPy, tPTheta = self.calculate_trajactory()

        self.update_prediction_model(
            self.RstateX, self.RstateY, self.RstateTheta, self.RstateVelocity, self.RstateW)

        I = 0
        alpha, Jsteps, Jgrad, Jgrad_prev = self.init_optmizer()

        self.speed_saturation()
        
        # Initial cost function value
        # Jatual = COST_FUNCTION2(Sinbo.x, Sinbo.y, Sinbo.theta, Sinbo.v, ...
        #     Sinbo.w, Uref, tPX, tPY, yPTheta, N1, N2, Nu, L1, L2, L3);
        Jatual = self.calculate_cust_function()
        Jbest = Jatual

        while (I < self.max_iterations) and (self.actual_cost > self.eta):

            # Calcula os passos das entradas
            Usteps = calculate_Usteps(self.velocity_reference, self.control_horz, self.delta)

            # Faz o cálculo de todos 'J' e 'U' para um horizonte de predição
            # de controle Nu
            '''
            ver a explicação dos 3 fors no tempo: 1:01:00
            '''
            for k in range(0, self.control_horz):
                for j in range(0,4): # Para percorrer os vetores J
                    # ATRIBUI AS VELOCIDADES PARA CADA PASSO DE 'U'

                    for n in range(0, self.control_horz):
                        if n == k:
                            Uaux[1, n+1] = Usteps[1, (j + 4*k)]
                            Uaux[2, n+1] = Usteps[2, (j + 4*k)]
                        else:
                            Uaux[1, n+1] = self.velocity_reference[1, 1]
                            Uaux[2, n+1] = self.velocity_reference[2, 1]
                   
                    # REINICIA A POSIÇÃO INICIAL DO ROBÔ PARA CADA ITERAÇÃO
                    self.update_prediction_model(
                        self.RstateX, self.RstateY, self.RstateTheta, self.RstateVelocity, self.RstateW)
                    
                    # Wheels velocity saturation # Uref, d, Nu, Vmax
                    Uaux = scale_for_saturation(Uaux, self.wheels_distance, self.control_horz, self.max_velocity)

                    # CALCULA O 'J' DA ITERAÇÃO
                    # J = COST_FUNCTION2(Sinbo.x, Sinbo.y, Sinbo.theta, Sinbo.v, ...
                    # Sinbo.w, Uaux, tPX, tPY, yPTheta, N1, N2, Nu, L1, L2, L3)
                    J = self.calculate_cust_function()
                    Jsteps[j + 4*k, I] = J # Vetor de passos
 
            '''Cálculo do gradiente de J baseado nos Jsteps'''
            for h in range(0, self.control_horz):
                Jgrad_prev[2*h+1, 1] = Jgrad[2*h+1, 1]
                Jgrad[2*h+1, 1] = Jsteps[4*h+1, 1] - Jsteps[4*h+2, 1]
                
                Jgrad_prev[2*h+2, 1] = Jgrad[2*h+2, 1]
                Jgrad[2*h+2, 1] = Jsteps[4*h+3, 1] - Jsteps[4*h+4, 1]

            # COM OS GRADIENTES DE TODOS OS J CALCULADOS, INICIAMOS O CÁLCULO
            # DO GRADINETE CONJUGADO (achar o mínimo de J) - Polak & Bi

            d1 = [0, 0]
            x1 = d1

            for z in range(0, self.control_horz):# 0:1:Nu-1
                d1[0] = Jgrad[2*z + 1, 1]
                d1[1] = Jgrad[2*z + 2, 1]

                x1[0] = (Uref[1, z+1] - alpha*d1[0] )
                x1[1] = (Uref[1, z+1] - alpha*d1[1] )

                Jgrad_prev[2*z+1, 1] = Jgrad[2*z+1, 1]
                Jgrad[2*z+1, 1] = Jsteps[4*z+1, 1] - Jsteps[4*z+2, 1]

                Jgrad_prev[2*z+2, 1] = Jgrad[2*z+2, 1]
                Jgrad[2*z+2, 1] = Jsteps[4*z+3, 1] - Jsteps[4*z+4, 1]
                
                beta = 0

                if ( Jgrad[2*z+1, 1] >= self.eta ) or ( Jgrad[2*z+2, 1] >= self.eta ):
                    t1 = Jgrad[2*z+1, 1] - Jgrad_prev[2*z+1, 1]
                    t2 = Jgrad[2*z+2, 1] - Jgrad_prev(2*z+2, 1)

                    a1 = Jgrad(2*z+1, 1)*t1
                    a2 = Jgrad(2*z+2, 1)*t2

                    b1 =  Jgrad_prev[2*z+1, 1] - Jgrad_prev[2*z+1, 1] 
                    b2 =  Jgrad_prev[2*z+2, 1] - Jgrad_prev[2*z+2, 1]

                    beta = (a1+a2)/(b1+b2)

                Uref[1, z+1] = x1[1] + alpha*(-Jgrad[2*z+1, 1]) + beta* Jgrad_prev[2*z+1, 1]
                Uref[2, z+1] = x1[2] + alpha*(-Jgrad[2*z+2, 1]) + beta* Jgrad_prev[2*z+2, 1]
            
            '''
                Reinicia a posição inicial do robô que será usada
                no próximo loop de controle
                1. Atualiza o modelo
                2. Satura a velocidade das rodas
                3. Calcula o novo valor da função custo
            '''
            self.update_prediction_model(TRsx, TRsy, TRst, TRsv, TRsw)
            # SinRob.x = TRsx
            # SinRob.y = TRsy
            # SinRob.theta = TRst
            # SinRob.v = TRsv
            # SinRob.w = TRsw

            '''
            SATURA A VELOCIDADE DAS RODAS QUE SERÃO USADAS NO 
            PRÓXIMO LOOP DE CONTROLE. scale_for_saturation(Uref, d, Nu, Vmax)
            '''
            Uref = scale_for_saturation(Uref, WHEELS_DISTANCE, CONTROL_HORZ, V_MAX)

            ''' Recalcula o J atual usado no próximo loop de controle
            Jatual = COST_FUNCTION2(Sinbo.x, Sinbo.y, Sinbo.theta, Sinbo.v, ...
            Sinbo.w, Uref, tPX, tPY, yPTheta, N1, N2, Nu, L1, L2, L3) '''

            Jatual = self.calculate_cust_function()

            if Jatual < Jbest:
                Jbest = Jatual
                Ubest = Uref

            I = I + 1
        
        Vout_MPC = Ubest[1, 1]
        Wout_MPC = Ubest[2, 1]

        return Vout_MPC, Wout_MPC
