#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import tf
import numpy as np
import matplotlib.pyplot as plt

class RobotMove(object):

    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vel_msg = Twist()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=1)
        self.odom_msg = Odometry()
        self.v_speed = 0.1
        self.w_speed = 0.1
 
    def odometry_callback(self, odom_data):
        self.odom_msg = odom_data

    def move2goal(self, x_goal):
        print("moving")
        self.vel_msg.angular.x = self.w_speed
        while (self.odom_msg.pose.pose.position.x < x_goal):
            self.vel_msg.linear.x = self.v_speed
            self.vel_pub.publish(self.vel_msg)

        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)
        



def main():
    print("Init Node")
    rospy.init_node('square_path_node', anonymous=True)
    robot_move = RobotMove()

    robot_move.move2goal(4)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()