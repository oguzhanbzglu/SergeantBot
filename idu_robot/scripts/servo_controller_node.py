#!/usr/bin/env python3

import rospy
from Adafruit_PCA9685 import PCA9685
import time

# Initialise the PCA9685 using the default address (0x40).
pwm = PCA9685()

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def move_servo():
    rospy.loginfo("Moving servo on channel 0")

    # Move servo on channel O between extremes.
    pwm.set_pwm(0, 0, servo_min)
    time.sleep(1)
    pwm.set_pwm(0, 0, servo_max)
    time.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('servo_controller', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            move_servo()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

