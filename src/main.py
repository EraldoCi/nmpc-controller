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

class Node:
    def __init__(self) -> None:
        """
        Initialze a ros node
        """

        # Node creation
        rospy.init_node('NMPC_Robot')
        # Publisher which will publish to the topic 'cmd_vel_mux/input/navi'
        self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_position)
        
        self.pose = Pose()
        self.rate = rospy.Rate(2)
        self.move = Twist()
    
    '''
    def create_node(self):
        rospy.init_node(self.node_name)
        pub = rospy.Publisher(self.topic_name, Twist, queue_size=1)
        rate = rospy.Rate(2)

        move = Twist() # defining the way we can allocate the values
        move.linear.x = 0.0 # allocating the values in x direction - linear
        move.angular.z = 0.0  # allocating the values in z direction - angular

        return pub, rate, move
    '''

    def update_position(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

        # rospy.loginfo(f'X position: {data.pose.pose}')
        rospy.loginfo(f'X position: {self.pose.x}')


    def move_to_goal(self):
        # Initialize linear & angular velocities
        self.move.linear.x = 0
        self.move.linear.y = 0
        self.move.linear.z = 0
        self.move.angular.x = 0
        self.move.angular.y = 0
        self.move.angular.z = 0

        # path iterator
        index = 0
        max_iter = len(new_coordinates) - 1

        xrefA = new_coordinates[0][index]
        yrefA = new_coordinates[1][index]
        thetarefA = self.pose.theta
        vrefA = self.move.linear.x
        wrefA = self.move.angular.z

        while not rospy.is_shutdown() and index < max_iter:

            xref = new_coordinates[0][index]
            yref = new_coordinates[1][index]

            inputRef = ControllerInput(
                xref = xref,
                yref = yref,
                RstateX = self.pose.x,
                RstateY = self.pose.y,
                RstateTheta = self.pose.theta,
                RstateVelocity = self.move.linear.x,
                RstateW = self.move.angular.z,
                xrefA = xrefA,
                yrefA = yrefA,
                thetarefA = thetarefA,
                vrefA = vrefA,
                wrefA = wrefA
            )
            nmpc = NMPC_Controller(input=inputRef)
            nmpc.init_optmizer()
            
            self.move.linear.x, self.move.angular.z = nmpc.start_optmizer()
            
            self.velocity_publisher.publish(self.move)
            self.rate.sleep()

            # Previous params
            xrefA = xref
            yrefA = yref
            thetarefA = self.pose.theta
            vrefA = self.move.linear.x
            wrefA = self.move.angular.z

            index += 1
        return

'''
def start_robot():
    robot_node = Node('NMPC_Robot', '/cmd_vel_mux/input/navi')
    pub, rate, moviment = robot_node.create_node()

    rospy.Subscriber('/odom', Odometry, robot_node.callback_Actual_Position)

    Vout_MPC, Wout_MPC = nmpc.start_optmizer()

    index = 0
    # Start Control
    while not rospy.is_shutdown():
        
        xref = new_coordinates[0][index]
        yref = new_coordinates[1][index]

        inputRef = ControllerInput(
            xref = xref,
            yref = yref,
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
        nmpc.init_optmizer()

        pub.publish(moviment)
        rospy.loginfo(f'Speed: {moviment.linear}')
        # print(f'\nLinear speed: {moviment.linear.x}')
        rate.sleep()

        index += 1
'''

if __name__ == '__main__':
    print(f'{__file__} start!!')
    '''
    start_point_x = 60.0  # [m]
    start_point_y = 40.0  # [m]
    final_point_x = 50.0  # [m]
    final_point_y = 320.0  # [m]
    grid_size = 3.0  # [m]
    robot_radius = 1.0  # [m]

    # Read bmp file
    mapImage = Image.open('/home/gustavo/catkin_workspace/src/src/robotic_part2/assets/mapaexemplo.bmp')
    imgGray = ImageOps.grayscale(mapImage)
    mapToArrray = np.array(imgGray)

    height, _  = mapToArrray.shape
    map_grid = list(mapToArrray)

    new_ox, new_oy = [], []

    # populates X and Y axis
    for i in range(height-1, 0, -1):
        for k,j in enumerate(map_grid[i]):
            if j == 0:
                # pixel with 0 value means it's an obstacle
                new_ox.append(i)
                new_oy.append(k)

    a_star = AStarPlanner(new_ox, new_oy, grid_size, robot_radius)
    rx, ry = a_star.planning(start_point_x, start_point_y, final_point_x, final_point_y)
    
    # Final plot
    plt.plot(new_ox, new_oy, ".k")
    plt.plot(start_point_x, start_point_y, "og")
    plt.plot(final_point_x, final_point_y, "xb")
    plt.grid(True)
    plt.axis("equal")

    # new_rx = np.array(rx)/50
    # new_ry = np.array(ry)/50
    # print(new_rx, new_ry)
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()
    '''

    new_robot = Node()
    new_robot.move_to_goal()
    