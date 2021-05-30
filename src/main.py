#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from PIL import Image, ImageOps
import numpy as np

from aStar import AStarPlanner 
from nmpcController import NMPC_Controller
from utils.controllerInput import ControllerInput

'''
1. Gerar o caminho com o aStar
2. Enviar o array com as coordenadas para o robo
3. Executar o controle
'''

class Node:
    def __init__(self, node_name:str, topic_name:str) -> None:
        """
        Initialze a ros node
        """
        self.node_name = node_name # topic_publisher
        self.topic_name = topic_name # /cmd_vel_mux/input/navi

    def create_node(self):
        rospy.init_node(self.node_name)
        pub = rospy.Publisher(self.topic_name, Twist, queue_size=1)
        rate = rospy.Rate(2)

        move = Twist() # defining the way we can allocate the values
        move.linear.x = 0.9 # allocating the values in x direction - linear
        move.angular.z = 0.2  # allocating the values in z direction - angular

        return pub, rate, move

def start_robot():
    robot_node = Node('NMPC_Robot', '/cmd_vel_mux/input/navi')
    pub, rate, moviment = robot_node.create_node()

    # Start Control
    while not rospy.is_shutdown():
        pub.publish(moviment)
        rate.sleep()

if __name__ == '__main__':
    print(f'{__file__} start!!')

    '''
    start_point_x = 50.0  # [m]
    start_point_y = 50.0  # [m]
    final_point_x = 200.0  # [m]
    final_point_y = 200.0  # [m]
    grid_size = 3.0  # [m]
    robot_radius = 1.0  # [m]

    # Read bmp file
    mapImage = Image.open('/home/gustavo/catkin_workspace/src/src/robotic_part2/assets/mapaexemplo.bmp')
    imgGray = ImageOps.grayscale(mapImage)
    mapToArrray = np.array(imgGray)

    height, _  = mapToArrray.shape
    map_grid = list(mapToArrray)

    #  X & Y axis
    new_ox, new_oy = [], []

    # populates X and Y axis
    for i in range(height):
        for k,j in enumerate(map_grid[i]):
            if j == 0:
                # pixel with 0 value means it's an obstacle
                new_ox.append(k)
                new_oy.append(i)

    a_star = AStarPlanner(new_ox, new_oy, grid_size, robot_radius)
    rx, ry = a_star.planning(start_point_x, start_point_y, final_point_x, final_point_y)

    # Final plot
    plt.plot(new_ox, new_oy, ".k")
    plt.plot(start_point_x, start_point_y, "og")
    plt.plot(final_point_x, final_point_y, "xb")
    plt.grid(True)
    plt.axis("equal")

    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()
    '''

    # start_robot()
    inputRef = ControllerInput(
        xref = 0.5,
        yref = 0.5,
        RstateX = 0.5,
        RstateY = 0.5,
        RstateTheta = 0.5,
        RstateVelocity = 0.5,
        RstateW = 0.5,
        xrefA = 0.5,
        yrefA = 0.5,
        thetarefA = 0.5,
        vrefA = 0.5,
        wrefA = 0.5
    )
    nmpc = NMPC_Controller(input=inputRef)
    # print(nmpc.__str__())
    