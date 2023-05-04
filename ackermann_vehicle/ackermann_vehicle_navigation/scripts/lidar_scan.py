#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class ObsDetection(): #main class
   
    def __init__(self): #main function
        self.circle = Twist() #create object of twist type  
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback) #subscribe message 
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message

    def laser_callback(self,msg):
        self.distance = 1.0 #distance of between robot and obs
        list_of_angle = list()
        #print(len(msg.ranges))   
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i*msg.angle_increment #getting angles for each scan line
            if angle>-0.79 and angle<0.79: #45 degree is ~0.78rad
                #print(i) #index of angels
                list_of_angle.append(msg.ranges[i])
        min_range = (min(list_of_angle))

        if min_range<self.distance:
            #print("WARNING! OBSTACLE DETECTED!!!!!")
            rospy.loginfo("WARNING! OBSTACLE IS DETECTED!!!!!")
            self.circle.linear.x = 0.1 # stop
            self.circle.angular.z = 0.5 # rotate counter-clockwise
            

        else:
            self.circle.linear.x = 0.5 # go (linear velocity)
            self.circle.angular.z = 0.0 # rotate (angular velocity)
            rospy.loginfo("MOVING!") #state situation constantly
        self.pub.publish(self.circle) # publish the move object

    def odometry(self, msg): #function for odometry
        msg.pose.pose #print position and orientation of turtlebot

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node') #initilize node
    ObsDetection() #run class
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        rospy.on_shutdown(ObsDetection)