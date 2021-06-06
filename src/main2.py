#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from PIL import Image, ImageOps
import numpy as np

from aStar import AStarPlanner 
from nmpcController import NMPC_Controller
from utils.inputClasses import ControllerInput
from utils.test.coordinates import new_coordinates

'''
1. Gerar o caminho com o aStar
2. Enviar o array com as coordenadas para o robo
3. Executar o controle
'''

class Robot_Node():
    def __init__(self):

        # Publisher
        self.pub_control = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.update_position)
        self.rate = rospy.Rate(2)

        self.x = 0
        self.y = 0
        self.theta = 0
    
    def update_position(self, data):
        self.x = round(data.pose.pose.position.x, 4)
        self.y = round(data.pose.pose.position.y, 4)
        self.theta = round(data.pose.pose.orientation.z, 4)

        rospy.loginfo(f'X position: {self.x}\nY position: {self.y}\nTheta: {self.theta}')
    

    def move_to_goal(self):
        move = Twist()
        move.linear.x = 0.5
        move.angular.z = 0.5

        self.pub_control.publish(move)
        self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node('NMPC_Robot')
    robot_node = Robot_Node()
    
    # index = 0

    # moviment = Twist()
    # moviment.linear.x = 0.5
    # moviment.linear.y = 0
    # moviment.linear.z = 0
    # moviment.angular.x = 0
    # moviment.angular.y = 0
    # moviment.angular.z = 0

    # xrefA = new_coordinates[0][index]
    # yrefA = new_coordinates[1][index]
    # thetarefA = robot_node.theta
    # vrefA = moviment.linear.x
    # wrefA = moviment.angular.z

    while not rospy.is_shutdown():
        robot_node.move_to_goal()

        # xref = new_coordinates[0][index]
        # yref = new_coordinates[1][index]

        # inputRef = ControllerInput(
        #     xref = xref,
        #     yref = yref,
        #     RstateX = robot_node.x,
        #     RstateY = robot_node.y,
        #     RstateTheta = robot_node.theta,
        #     RstateVelocity = moviment.linear.x,
        #     RstateW = moviment.angular.z,
        #     xrefA = xrefA,
        #     yrefA = yrefA,
        #     thetarefA = thetarefA,
        #     vrefA = vrefA,
        #     wrefA = wrefA
        # )

        # nmpc = NMPC_Controller(input=inputRef)
        # rospy.loginfo(f'X position: {xref}\n')
        # print(f'Linear velocity: {moviment.linear.x}\n Angular velocity: {moviment.angular.z}')
        
        # Start control
        # moviment.linear.x, moviment.angular.z = nmpc.start_optmizer()
        
        # robot_node.pub_control.publish(moviment)
        # rospy.sleep(1)
        
        # Set previous reference
        # xrefA, yrefA = xref, yref
        # thetarefA, vrefA, wrefA = robot_node.theta, moviment.linear.x, moviment.angular.z
        # index += 1


