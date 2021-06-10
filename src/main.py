#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

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
        # unique node (using anonymous=True).
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
        self.start_control_flag = 1
        # rospy.loginfo(data)

    def move2goal(self):
        """Moves the turtle to the goal."""
        vel_msg = Twist()

        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0.5 # m/s
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.0 # rad/s

        # Starting point reference
        x_ref = 0.0
        y_ref = -5.0

        # Previous Reference
        x_prev_ref = 0.0
        y_prev_ref = 0.0
        theta_prev_ref = self.theta
        vrefA = 0.5
        wrefA = 0.0

        while not rospy.is_shutdown():
            # if self.start_control_flag:
            inputRef = ControllerInput(
                xref=x_ref,
                yref=y_ref,
                RstateX=self.x_position,
                RstateY=self.y_position,
                RstateTheta=self.theta,
                RstateVelocity=vel_msg.linear.x,
                RstateW=vel_msg.angular.z,
                xrefA=x_prev_ref,
                yrefA=y_prev_ref,
                thetarefA=theta_prev_ref,
                vrefA=vrefA,
                wrefA=wrefA
            )
            # print(inputRef)

            # rospy.loginfo(f'REF_Previous:{x_prev_ref, y_prev_ref}')
            # rospy.loginfo(f'ACTUAL: {self.x_position, self.y_position}')
            nmpc = NMPC_Controller(inputRef)
            tPx, tPY, tPTheta = nmpc.test_create_mini_path()
            XREF = tPx[0]

            # print(f'X TRAJECTORY: {tPx}\nY TRAJECTORY: {tPY}')


            self.velocity_publisher.publish(vel_msg)

            new_v, new_w = nmpc.start_optmizer()
            new_v = round(new_v, 4)
            new_w = round(new_w, 4)

            # print(new_v, new_w)
            # rospy.loginfo(
            #     f'X: {self.x_position}, Y: {self.y_position}, THETA: {self.theta}')

            x_prev_ref = self.x_position
            y_prev_ref = self.y_position
            theta_prev_ref = self.theta
            vrefA = vel_msg.linear.x
            wrefA = vel_msg.angular.z

            self.start_control_flag = 0

            '''Update the linear & angular velocity'''
            vel_msg.linear.x = new_v
            vel_msg.angular.z = new_w

            self.rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    try:
        turtlebot = TurtleBot()
        turtlebot.move2goal()
    except rospy.ROSInterruptException:
        pass
