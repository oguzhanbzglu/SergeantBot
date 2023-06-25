#!/usr/bin/env python3

import time
import smbus
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis

ADDR = None
bus = None
IMU_FRAME = None


def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = read_word_2c(0x3B) / 16384.0
    accel_y = read_word_2c(0x3D) / 16384.0
    accel_z = read_word_2c(0x3F) / 16384.0
    
    # Calculate a quaternion representing the orientation
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Read the gyro vals
    #494.9114198473281, 0.7813587786259514, 1.5617938931297675]
    gyro_x = (read_word_2c(0x43)  / 131.0)+ 494.9114198473281-489.5
    gyro_y = ((read_word_2c(0x45) )/ 131.0)+ 0.7813587786259514-1.5
    gyro_z = ((read_word_2c(0x47) ) / 131.0) +1.5617938931297675-3.1
    
    # Load up the IMU message
    o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x 
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z
    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)


temp_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    # Wake up the sensor
    bus.write_byte_data(ADDR, 0x6B, 0)

    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    rospy.spin()

