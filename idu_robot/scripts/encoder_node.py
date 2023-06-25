#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32

ENCODER_PIN = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def encoder_callback():
    count = 0
    last_state = GPIO.input(ENCODER_PIN)
    pub = rospy.Publisher('/encoder_count', Int32, queue_size=10)
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        current_state = GPIO.input(ENCODER_PIN)
        if current_state != last_state:
            count += 1 
            rospy.loginfo("Encoder Count: {}".format(count))
            pub.publish(count)
        last_state = current_state
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('encoder_node')
        encoder_callback()
    finally:
        GPIO.cleanup()

