#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32

MOTOR_PIN = 4
PWM_FREQ = 100 #changable
PWM_DUTY_CYCLE = 90

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

pwm = GPIO.PWM(MOTOR_PIN, PWM_FREQ)
pwm.start(PWM_DUTY_CYCLE)

def motor_callback(data):
    pwm.ChangeDutyCycle(data.data)

if __name__ == '__main__':
    try:
        rospy.init_node('motor_node')
        rospy.Subscriber('/motor_control', Float32, motor_callback)
        rospy.spin()
    finally:
        pwm.stop()
        GPIO.cleanup()

