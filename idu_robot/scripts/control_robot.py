#!/usr/bin/env python3

#import the libraries
import rospy
from Adafruit_PCA9685 import PCA9685 #servo driver lib
import RPi.GPIO as GPIO #raspi pins
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist #to vel msg
import tty
import termios
import sys

#SERVO SETTINGS
#Initialise the PCA9685 usin the default address (0x40)
pwm_servo = PCA9685()

#Configure min and max servo pulse lengths
servo_min = 230
servo_max = 470

#Set frequency to 60Hz which is good for servos
pwm_servo.set_pwm_freq(60)

#DC SETTINGS
MOTOR_PIN = 4 #dc connection pin on raspi
PWM_FREQ = 500 #freq of ESC (Electronic Speed Controller)
PWM_DUTY_CYCLE = 0 #initial duty cycle of dc, it won't start when you run the code first

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm_dc = GPIO.PWM(MOTOR_PIN, PWM_FREQ)
pwm_dc.start(PWM_DUTY_CYCLE)

#Move servo on channel 0 to the specified position
def move_servo(servo_pose):
    pwm_servo.set_pwm(0, 0, servo_pose)


#Define a function to get the pressed key from the terminal:
def get_key():
    #Get the pressed key
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
        rospy.init_node('robot_control')
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        rate = rospy.Rate(10) #Hz
        twist = Twist() #create msg
        dc_duty_cycle = PWM_DUTY_CYCLE #which is zero
        servo_step = (servo_max-servo_min)//10 #define the servo step, in here we have 10 steps
        servo_pose = servo_min + (servo_max-servo_min) // 2 #Set initial servo pose
        print("""
        ***IDU ROBOT - SERGEANT***

           ___
     |     | |
    / \    | |
   |--o|===|-|
   |---|   |I|
  /     \  |D|
 |   T   | |U|
 |   U   |=| |
 |   R   | | |
 |_______| |_|
  |@| |@|  | |
___________|_|_
        ***IDU ROBOT - SERGEANT***
            """)
        print("Initial servo position: ", servo_pose)
        print("Initial dc velocity: ", dc_duty_cycle) 

        while not rospy.is_shutdown():
            
            key = get_key()
            if key.lower() == 'w':
                dc_duty_cycle = 90
                twist.linear.x  = dc_duty_cycle
                print("DC motor runs: ",dc_duty_cycle, "PWM" )
            elif key.lower() == 's':
                dc_duty_cycle = 0
                twist.linear.x = dc_duty_cycle
                print("DC motor stops: ", dc_duty_cycle, "PWM")
            
            elif key.lower() == "d":
                #Turn right by increasing the servo position
                if (servo_pose + servo_step) <= servo_max:
                    servo_pose += servo_step
                move_servo(servo_pose)
                twist.angular.z = servo_pose
                print("Servo position: ", servo_pose)
            elif key.lower() == "a":
                if (servo_pose - servo_step )>= servo_min:
                    servo_pose -= servo_step
                move_servo(servo_pose)
                twist.angular.z = servo_pose
                print("Servo position: ", servo_pose)
            
            else:
                #move servo to the center position
                move_servo(servo_min+(servo_max-servo_min)//2) #350
                twist.angular.z = servo_pose
                twist.linear.x  = dc_duty_cycle
            
            pub.publish(twist)
            pwm_dc.ChangeDutyCycle(dc_duty_cycle)
            rate.sleep()
    finally:
        pwm_dc.stop()
        #pwm stop
        GPIO.cleanup()