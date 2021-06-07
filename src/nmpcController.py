#! /usr/bin/env python
import numpy as np
import math

from utils.inputClasses import ControllerInput, \
    CostFunctionParams, TrajectoryInput
from utils.functions import calculate_Usteps, \
    calculate_mini_trajectory, cost_function, \
    scale_for_saturation, reversed_kinematic

# Controller constraint params initialization
V_MAX = 0.4 # Vmax
WHEELS_DISTANCE = 0.23 # d
PREDICT_HORZ = 1 # N1
PREDICT_HORZ_END = 10 # N2
CONTROL_HORZ = 2 # Nu
GAIN_XY_ERROR = 0.05 # L1
GAIN_THETA_ERROR = 2.5 # L2
GAIN_DELTA_ERROR = 0.85 # L3
ETA = 0.1
MAX_ITERATIONS = 15 
DELTA = 0.1
ALPHA =  0.015 # Passo do otimizador do gradiente descendente

'''
xref, yref, Rsx, Rsy, Rst, Rsv, ...
	Rsw, xrefA, yrefA, thetarefA, vrefA, wrefA

Lembrar de fazer a realimentação de
* posição de x
* posição de y
* theta do robô
* v do robô
* w do robô

Os valores de referência anterior precisam de realimentação
em conjunto com um atraso.

'''

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

        self.Ubest, self.Uref, self.Uaux, self.prediction_model = self.init_predict_model()

    def __str__(self):# used for debuging
        return f'''
            X pose:{self.RstateX}, 
            Y pose: {self.RstateY},
            V: {self.RstateVelocity}
            
            '''


    def init_optmizer(self):
        '''Parâmetros do Otimizador
        alpha: Passo do otimizador do gradiente descendente.

        Inicializa os vetores do Otimizador
        Jsteps: Vetor de passos J 1x8. Onde J é a função custo.
        Jgrad: Vetor de passos do Gradiente de J 4x1.
        Jgrad_prev: Vetor de passos do Gradiente Anterior de J 4x1.
        '''
        # alpha = alpha
        Jsteps = np.zeros((1, CONTROL_HORZ*4))
        Jgrad = np.zeros((1, CONTROL_HORZ*2))
        Jgrad_prev = np.zeros((1, CONTROL_HORZ*2))

        return Jsteps, Jgrad, Jgrad_prev


    def init_predict_model(self):
        '''Inicializa o modelo do preditor.
        
        Retorna:
        Ubest: Melhor U control_horz = 2x2. 2 pq são V e W(ohmega) e porque são os dois horizontes de controle.
        Uref: Saída do controle de referência. U final JxNu = 2x2.
        Uaux: Saída do controle auxiliar que vai servir para o otimizador.
        SinRob: .
        '''
        Ubest = np.zeros((2,CONTROL_HORZ)) 
        Uref = np.zeros((2, CONTROL_HORZ)) 
        Uaux = np.zeros((2, CONTROL_HORZ))

        SinRob = {'x': 0, 'y': 0, 'theta': 0, 'v': 0, 'w': 0}

        return Ubest, Uref, Uaux, SinRob
    

    def init_controller_variables(self):
        '''
        Um dos componentes para inicializar as variáveis
        do controlador.
        '''
        for i in range(0, CONTROL_HORZ-1):
            self.Uref[0, i] = self.vrefA
            self.Uref[1, i] = self.wrefA
        
        return


    def calculate_trajactory(self, trajectoty_input: TrajectoryInput):
        '''
        Olhar no minuto 40
        '''
        tPx, tPY, tPTheta = calculate_mini_trajectory(trajectoty_input)
        # print(f'calculate_trajactory: {tPx}\ntPY: {tPY}\ntPTheta: {tPTheta}')
        # tPx, tPY, tPTheta é um vetor com 10 posições
        return tPx, tPY, tPTheta


    def update_prediction_model(self, Rsx, Rsy, Rst, Rsv, Rsw):
        ''' Atualização do modelo de predição.
        Rsx: robot X state. 
        Rsy: robot Y state.
        Rst: robot Theta state.
        Rsv: robot linear velocity state.
        Rsw: robot angular velocity state.
        '''
        self.prediction_model['x'] = Rsx
        self.prediction_model['y'] = Rsy
        self.prediction_model['theta'] = Rst
        self.prediction_model['v'] = Rsv
        self.prediction_model['w'] = Rsw

        # print(f'update_prediction_model: {self.prediction_model}')

    def speed_saturation(self, Uref):
        '''
            Satura a velocidade.
        '''
        new_output_ref = scale_for_saturation(Uref, WHEELS_DISTANCE, CONTROL_HORZ, V_MAX)
        # print(f'speed_saturation: {new_output_ref}')
        return new_output_ref


    def calculate_cust_function(self, Uref, tPX, tPY, tPTheta) -> float:
        # lembrar de alterar os campos: 
        #  x_ref_pos, y_ref_pos, Theta_ref
        cost_input = CostFunctionParams(
            x_position = self.prediction_model['x'], 
            y_position = self.prediction_model['y'], 
            robot_theta = self.prediction_model['theta'] , 
            linear_v = self.prediction_model['v'],
            angular_v_w = self.prediction_model['w'],
            velocities_vector = Uref, # Uref
            x_ref_pos =  tPX, # array x 10
            y_ref_pos = tPY, # array x 10
            Theta_ref = tPTheta, # array x 10
            # predict_horz = PREDICT_HORZ,
            predict_horz_end = PREDICT_HORZ_END,
            control_horz = CONTROL_HORZ, 
            gain_xy_error = GAIN_XY_ERROR, 
            gain_theta_error = GAIN_THETA_ERROR,
            gain_delta_control = GAIN_DELTA_ERROR,
        )
        
        return cost_function(cost_input) # self.actual_cost = cost_function(cost_input) 


    def start_optmizer(self):
        
        self.vrefA, self.wrefA, self.thetarefA = reversed_kinematic(
            self.RstateVelocity, 0.0, self.RstateX, self.RstateY)

        Jsteps, Jgrad, Jgrad_prev = self.init_optmizer()

        self.init_controller_variables()

        # Cálculo da trajatória de referência (mini trajetoria)
        # lTi, Rsx, Rsy,  Rst, xrefA, yrefA, tetarefA, ...
		#vrefA, wrefA, N2, xref, yref
        trajectoty_input = TrajectoryInput(
            Rx = self.RstateX,
            Ry = self.RstateY,
            Rt = self.RstateTheta,
            trajX = self.xrefA,
            trajY = self.yrefA,
            trajTeta = self.thetarefA,
            V = self.vrefA,
            W = self.wrefA,
            N2 = PREDICT_HORZ_END,
            trajXp = self.xref,
            trajYp = self.yref,
        )

        tPX, tPY, tPTheta = self.calculate_trajactory(trajectoty_input)
        # print(f'\tTPX:{tPX}')
        # tPXa = tPX[1]
        # tPYa = tPY[1]
        # tPTa = tPTheta[1]

        self.update_prediction_model(
            self.RstateX, self.RstateY, self.RstateTheta, self.RstateVelocity, self.RstateW)

        I = 0
        

        self.Uref = self.speed_saturation(self.Uref)
        # print(f'speed_saturation: {self.Uref}')
        
        '''Initial cost function value
        Jatual = COST_FUNCTION2(Sinbo.x, Sinbo.y, Sinbo.theta, Sinbo.v, ...
        Sinbo.w, Uref, tPX, tPY, yPTheta, N1, N2, Nu, L1, L2, L3);'''
        
        Jatual = self.calculate_cust_function(self.Uref, tPX, tPY, tPTheta)
        Jbest = Jatual + 5
        # print(f'JBEST: {Jbest}\n')

        while (I < MAX_ITERATIONS) and (Jatual > ETA):

            # Calcula os passos das entradas
            Usteps = calculate_Usteps(self.Uref, CONTROL_HORZ, DELTA)

            # Faz o cálculo de todos 'J' e 'U' para um horizonte de predição
            # de controle Nu
            '''
            ver a explicação dos 3 fors no tempo: 1:01:00
            '''
            for k in range(0, CONTROL_HORZ):
                for j in range(0, 4): # Para percorrer os vetores J
                    # ATRIBUI AS VELOCIDADES PARA CADA PASSO DE 'U'

                    for n in range(0, CONTROL_HORZ):
                        if n == k:
                            self.Uaux[0, n] = Usteps[0, (j + 4*k)]
                            self.Uaux[1, n] = Usteps[1, (j + 4*k)]
                        else:
                            self.Uaux[0, n] = self.Uref[0, 0]
                            self.Uaux[1, n] = self.Uref[1, 0]
                   
                    # REINICIA A POSIÇÃO INICIAL DO ROBÔ PARA CADA ITERAÇÃO
                    self.update_prediction_model(
                        self.RstateX, self.RstateY, self.RstateTheta, self.RstateVelocity, self.RstateW)
                    
                    # Wheels velocity saturation # Uref, d, Nu, Vmax
                    self.Uaux = scale_for_saturation(self.Uaux, WHEELS_DISTANCE, CONTROL_HORZ, V_MAX)

                    # CALCULA O 'J' DA ITERAÇÃO
                    # J = COST_FUNCTION2(Sinbo.x, Sinbo.y, Sinbo.theta, Sinbo.v, ...
                    # Sinbo.w, Uaux, tPX, tPY, yPTheta, N1, N2, Nu, L1, L2, L3)
                    J = self.calculate_cust_function(self.Uaux, tPX, tPY, tPTheta)
                    Jsteps[0, j + 4*k] = J # Vetor de passos
 
            '''Cálculo do gradiente de J baseado nos Jsteps'''
            for h in range(0, CONTROL_HORZ):
                Jgrad_prev[0, 2*h] = Jgrad[0, 2*h+1] # 1x4
                Jgrad[0, 2*h] = Jsteps[0, 4*h] - Jsteps[0, 4*h+1]
                
                Jgrad_prev[0, 2*h + 1] = Jgrad[0, 2*h+1]
                Jgrad[0, 2*h + 1] = Jsteps[0, 4*h+2] - Jsteps[0, 4*h+3]

            # COM OS GRADIENTES DE TODOS OS J CALCULADOS, INICIAMOS O CÁLCULO
            # DO GRADINETE CONJUGADO (achar o mínimo de J) - Polak & Bi

            d1 = [0, 0]
            x1 = [0, 0]

            for z in range(0, CONTROL_HORZ):# 0:1:Nu-1
                d1[0] = Jgrad[0, 2*z]
                d1[1] = Jgrad[0, 2*z + 1]

                x1[0] = (self.Uref[0, z] - ALPHA*d1[0] )
                x1[1] = (self.Uref[0, z] - ALPHA*d1[1] )

                Jgrad_prev[0, 2*z+1] = Jgrad[0, 2*z+1]
                Jgrad[0, 2*z+1] = Jsteps[0, 4*z+1] - Jsteps[0, 4*z+2]

                Jgrad_prev[0, 2*z+1] = Jgrad[0, 2*z+1]
                Jgrad[0, 2*z+1] = Jsteps[0, 4*z+2] - Jsteps[0, 4*z+3]
                
                beta = 0

                if ( Jgrad[0, 2*z+1] >= ETA ) or ( Jgrad[0, 2*z+1] >= ETA ):
                    t1 = Jgrad[0, 2*z] - Jgrad_prev[0, 2*z]
                    t2 = Jgrad[0, 2*z+1] - Jgrad_prev[0, 2*z+1]

                    a1 = Jgrad[0, 2*z]*t1
                    a2 = Jgrad[0, 2*z+1]*t2

                    b1 =  Jgrad_prev[0, 2*z]**2 
                    b2 =  Jgrad_prev[0, 2*z+1]**2
                  
                    beta = (a1+a2)/(b1+b2)


                self.Uref[0, z] = x1[0] + ALPHA*(-Jgrad[0, 2*z]) + beta* Jgrad_prev[0, 2*z]
                self.Uref[1, z] = x1[1] + ALPHA*(-Jgrad[0, 2*z+1]) + beta* Jgrad_prev[0, 2*z+1]
            
            '''
                Reinicia a posição inicial do robô que será usada
                no próximo loop de controle
                1. Atualiza o modelo
                2. Satura a velocidade das rodas
                3. Calcula o novo valor da função custo
            '''
            self.update_prediction_model(
                self.RstateX, self.RstateY, self.RstateTheta, self.RstateVelocity, self.RstateW)
            # SinRob.x = TRsx
            # SinRob.y = TRsy
            # SinRob.theta = TRst
            # SinRob.v = TRsv
            # SinRob.w = TRsw

            '''
            SATURA A VELOCIDADE DAS RODAS QUE SERÃO USADAS NO 
            PRÓXIMO LOOP DE CONTROLE. scale_for_saturation(Uref, d, Nu, Vmax)
            '''
            self.Uref = scale_for_saturation(self.Uref, WHEELS_DISTANCE, CONTROL_HORZ, V_MAX)

            ''' Recalcula o J atual usado no próximo loop de controle
            Jatual = COST_FUNCTION2(Sinbo.x, Sinbo.y, Sinbo.theta, Sinbo.v, ...
            Sinbo.w, Uref, tPX, tPY, yPTheta, N1, N2, Nu, L1, L2, L3) '''

            Jatual = self.calculate_cust_function(self.Uref, tPX, tPY, tPTheta)

            if Jatual < Jbest:
                Jbest = Jatual
                self.Ubest = self.Uref

            I = I + 1
            # print(f'J_ATUAL = {Jatual}')
        
        Vout_MPC = self.Ubest[0, 0]
        Wout_MPC = self.Ubest[1, 0]

        '''
        A saída do controlador vai se tornar a velocidade 
        angular e linear de referência (self.vrefA, self.wrefA) 
        '''

        print(f'FINAL DO CONTROLE {self.Ubest}')

        return Vout_MPC, Wout_MPC
