#!/usr/bin/env python3

import rospy
from Adafruit_PCA9685 import PCA9685
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import tty
import termios
import sys

# Initialise the PCA9685 using the default address (0x40).
pwm = PCA9685()

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 500  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

MOTOR_PIN = 4
PWM_FREQ = 500
PWM_DUTY_CYCLE = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

pwm_dc = GPIO.PWM(MOTOR_PIN, PWM_FREQ)
pwm_dc.start(PWM_DUTY_CYCLE)

def motor_callback(data):
    pwm_dc.ChangeDutyCycle(data.data)

def move_servo(servo_pos):
    # Move servo on channel 0 to the specified position
    pwm.set_pwm(0, 0, servo_pos)

def get_key():
    # Get the pressed key
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

if __name__ == '__main__':
    try:
        rospy.init_node('teleop_key')
        rospy.Subscriber('/motor_control', Float32, motor_callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(10)  # 10Hz
        twist = Twist()
        dc_duty_cycle = PWM_DUTY_CYCLE
        servo_step = (servo_max - servo_min) // 10  # Define the servo step
        servo_pos = servo_min + (servo_max - servo_min) // 2  # Set initial servo position
        print(servo_max - servo_min)
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'w':
                dc_duty_cycle = 90
                twist.linear.x = dc_duty_cycle
                twist.angular.z = servo_pos
                print("DC motor runs: ", dc_duty_cycle)
            elif key == 's':
                dc_duty_cycle = 0
                twist.linear.x = dc_duty_cycle
                twist.angular.z = servo_pos
                print("DC motor duty stopped: ", dc_duty_cycle)
            elif key == 'd':
                # Turn right by increasing the servo position
                if servo_pos + servo_step <= servo_max:
                    servo_pos += servo_step
                move_servo(servo_pos)
                twist.linear.x = dc_duty_cycle
                twist.angular.z = servo_pos
                print("Servo position: ", servo_pos)

            elif key == 'a':
                # Turn left by decreasing the servo position
                if servo_pos - servo_step >= servo_min:
                    servo_pos -= servo_step
                move_servo(servo_pos)
                twist.linear.x = dc_duty_cycle
                twist.angular.z = servo_pos
                print("Servo position: ", servo_pos)

            else:
                # Move servo to the center position
                move_servo(servo_min + (servo_max - servo_min) // 2)
                twist.linear.x = dc_duty_cycle
                twist.angular.z = servo_pos

            pub.publish(twist)
            pwm_dc.ChangeDutyCycle(dc_duty_cycle)
            rate.sleep()
    finally:
        pwm_dc.stop()
        # pwm.stop()
        GPIO.cleanup()
