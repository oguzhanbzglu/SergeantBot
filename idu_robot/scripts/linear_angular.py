#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class MotorVelocityCalculator:
    def __init__(self):
        self.encoder_count = 0
        self.last_encoder_count = 0
        self.last_time = rospy.Time.now()
        self.vel_pub = rospy.Publisher('/motor_velocity', Int32, queue_size=10)
        rospy.Subscriber('/encoder_count', Int32, self.encoder_callback)

        self.wheel_radius = 0.113 # meters

        # Set these values based on your specific motor and encoder
        self.encoder_ticks_per_rev = 40
        self.gear_ratio = 4

    def encoder_callback(self, msg):
        self.encoder_count = msg.data

    def calculate_velocity(self):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.last_time).to_sec()
        encoder_ticks = self.encoder_count - self.last_encoder_count
        velocity_ticks_per_sec = int(encoder_ticks / elapsed_time)
        velocity_m_per_sec = (2 * np.pi * self.wheel_radius * velocity_ticks_per_sec) / (self.encoder_ticks_per_rev * self.gear_ratio)
        self.vel_pub.publish(int(velocity_m_per_sec * 10)) # publish in m/s
        self.last_encoder_count = self.encoder_count
        self.last_time = current_time


class ImuReader:
    def __init__(self):
        self.angular_velocity = None
        rospy.Subscriber('/imu', Imu, self.imu_callback)

    def imu_callback(self, msg):
        self.angular_velocity = msg.angular_velocity.z


if __name__ == '__main__':
    rospy.init_node('motor_and_imu_node')

    mv = MotorVelocityCalculator()
    ir = ImuReader()

    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mv.calculate_velocity()

        twist = Twist()
        twist.linear.x = mv.vel_pub.get_num_connections() * 0.1  # Velocity in m/s
        twist.angular.z = ir.angular_velocity
        twist_pub.publish(twist)

        rate.sleep()

