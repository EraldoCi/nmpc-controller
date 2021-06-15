#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# import tf_conversions

from nmpcController import NMPC_Controller
from utils.inputClasses import ControllerInput


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return [yaw, pitch, roll]


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'NMPC_Robot' and make sure it is a
        rospy.init_node('NMPC_Robot', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
    
        self.x_position = 0.0
        self.y_position = 0.0
        self.theta = 0.0
        self.rate = rospy.Rate(1)
        self.start_control_flag = 0


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        # self.pose = data
        self.x_position = round(data.pose.pose.position.x, 4)
        self.y_position = round(data.pose.pose.position.y, 4)
        [yaw, _, _] = quaternion_to_euler(
            data.pose.pose.orientation.x, 
            data.pose.pose.orientation.y, 
            data.pose.pose.orientation.z, 
            data.pose.pose.orientation.w
        )
       
        self.theta = round(yaw, 4)
        print(f'(Reading) X: {data.pose.pose.position.x}\t Y:{data.pose.pose.position.y}')
        # self.theta = round(data.pose.pose.orientation.z, 4)

    def initiate_trajectory(
        self, x_ref, y_ref, vel_msg, 
        x_prev_ref, y_prev_ref, theta_prev_ref, vrefA, wrefA):

        inputRef = ControllerInput(
            xref = x_ref,
            yref = y_ref,
            RstateX = self.x_position,
            RstateY = self.y_position,
            RstateTheta = self.theta,
            RstateVelocity = vel_msg.linear.x,
            RstateW = vel_msg.angular.z,
            xrefA = x_prev_ref,
            yrefA = y_prev_ref,
            thetarefA = theta_prev_ref,
            vrefA = vrefA,
            wrefA = wrefA
        )

        # rospy.loginfo(f'X: {self.x_position} \tY: {self.y_position}\t Theta: {self.theta} ')
        nmpc = NMPC_Controller(inputRef)
        return nmpc.test_create_mini_path()
 

    def move2goal(self):
        """Moves the turtle to the goal."""
        vel_msg = Twist()

        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0.4 # m/s
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 1.5 # rad/s

        # Starting point reference
        goal_x = 1.0 
        goal_y = 1.0
        x_ref = 1.0
        y_ref = 1.0

        # Previous Reference
        x_prev_ref = 0.0
        y_prev_ref = 0.0
        theta_prev_ref = self.theta
        vrefA = 0.5
        wrefA = 0.0
        
        i = 0
        tPx, tPy, tPTheta = self.initiate_trajectory(
            x_ref, y_ref, vel_msg, 
            x_prev_ref, y_prev_ref, 
            theta_prev_ref, vrefA, wrefA
        )

        x_prev_ref = tPx[0]
        y_prev_ref = tPy[0]
        theta_prev_ref = tPTheta[0]

        print(f'X TRAJECTORY: {tPx}\nY TRAJECTORY: {tPy}\nTHETA TRAJ: {tPTheta}')
        print(f'ACTUAL THETA: {self.theta}')

        while not rospy.is_shutdown():
            
            if i >= 8:
                i = 0

                x_ref = goal_x
                y_ref = goal_y

                tPx, tPy, tPTheta = self.initiate_trajectory(
                    x_ref, y_ref, vel_msg, 
                    x_prev_ref, y_prev_ref, 
                    theta_prev_ref, vrefA, wrefA
                )
                # inputRef = ControllerInput(
                #     xref=x_ref,
                #     yref=y_ref,
                #     RstateX=self.x_position,
                #     RstateY=self.y_position,
                #     RstateTheta=self.theta,
                #     RstateVelocity=vel_msg.linear.x,
                #     RstateW=vel_msg.angular.z,
                #     xrefA=x_prev_ref,
                #     yrefA=y_prev_ref,
                #     thetarefA=theta_prev_ref,
                #     vrefA=vrefA,
                #     wrefA=wrefA
                # )

                # rospy.loginfo(f'X: {self.x_position} \tY: {self.y_position}\t Theta: {self.theta} ')
                # nmpc = NMPC_Controller(inputRef)
                # tPx, tPy, tPTheta = nmpc.test_create_mini_path()

                # print(f'X TRAJECTORY: {tPx}\nY TRAJECTORY: {tPy}\nTHETA TRAJ: {tPTheta}')
                # print(f'ACTUAL THETA: {self.theta}')
            
            # new_v, new_w = nmpc.start_optmizer()
            # new_v = round(new_v, 4)
            # new_w = round(new_w, 4)

            # print(new_v, new_w)
            # rospy.loginfo(
            #     f'X: {self.x_position}, Y: {self.y_position}, THETA: {self.theta}')
            
            # self.velocity_publisher.publish(vel_msg)
            # x_prev_ref = self.x_position
            # y_prev_ref = self.y_position
            # theta_prev_ref = self.theta
            # vrefA = vel_msg.linear.x
            # wrefA = vel_msg.angular.z
            

            # theta_prev_ref = self.theta
            # vel_msg.angular.z = 0.0


            '''Update the linear & angular velocity'''
            # vel_msg.linear.x = new_v
            # vel_msg.angular.z = new_w

            if i < 8:
                inputRef = ControllerInput(
                    xref = tPx[i],
                    yref = tPy[i],
                    RstateX = self.x_position,
                    RstateY = self.y_position,
                    RstateTheta = self.theta,
                    RstateVelocity = vel_msg.linear.x,
                    RstateW = vel_msg.angular.z,
                    xrefA = x_prev_ref,
                    yrefA = y_prev_ref,
                    thetarefA = theta_prev_ref,
                    vrefA = vrefA,
                    wrefA = wrefA
                )

                nmpc = NMPC_Controller(inputRef)
                new_v, new_w = nmpc.start_optmizer()
                new_v = round(new_v, 4)
                new_w = round(new_w, 4)

                print(f'(actual) X: {self.x_position}, Y: {self.x_position}, THETA: {self.theta}')
                print(f'(desired) X: {tPx[i]}, Y: {tPy[i]}')
                print(f'V: {vel_msg.linear.x}\tW: {vel_msg.angular.z}')

                x_prev_ref = tPx[i-1]
                y_prev_ref = tPy[i-1]
                theta_prev_ref = tPTheta[i-1]
                vrefA = vel_msg.linear.x
                wrefA = vel_msg.angular.z

                vel_msg.linear.x = new_v
                vel_msg.angular.z = new_w
                # vel_msg.angular.z = 0.0

                print(f'index: {i}')

                distance = math.sqrt((self.x_position - tPx[i])**2 + (self.y_position - tPy[i])**2)
                if distance < 0.3:
                    print(f'Distance: {distance}')
                    i+=1


            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    try:
        turtlebot = TurtleBot()
        turtlebot.move2goal()
    except rospy.ROSInterruptException:
        pass
