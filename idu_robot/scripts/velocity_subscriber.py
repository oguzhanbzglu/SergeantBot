#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu

class TwistCalculator:
    def __init__(self):
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/motor_velocity', Int32, self.linear_velocity_callback)
        rospy.Subscriber('/imu', Imu, self.angular_velocity_callback)

    def linear_velocity_callback(self, msg):
        self.linear_velocity = msg.data / 10.0

    def angular_velocity_callback(self, msg):
        self.angular_velocity = msg.angular_velocity.z

    def calculate_twist(self):
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = float(self.angular_velocity)
        self.twist_pub.publish(twist)
        

if __name__ == '__main__':
    rospy.init_node('twist_calculator_node')
    tc = TwistCalculator()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tc.calculate_twist()
        rate.sleep()

