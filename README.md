# SergeantBot

SergeantBot, I am working on an autonomous controlled vehicle project for my engineering department thesis using the ROS (Robot Operating System) software. The robot uses the Ackermann Steering method for its motion. Ackermann Steering is a type of steering mechanism commonly used in vehicles and mobile robots, including autonomous mobile robots. It is based on the principle that the inner wheel turns at a smaller angle than the outer wheel during a turn, resulting in both wheels following circular paths with different radii. This method provides stability and facilitates control of the vehicle or robot by preventing slipping. The main formulation of the Ackermann steering involves calculating the steering angles for the front wheels based on the desired turning radius and the vehicle's geometry. The steering angles are typically expressed as a function of the wheelbase (the distance between the front and rear axles) and the track width (the distance between the centers of the front wheels).

To apply Ackermann steering to an autonomous mobile robot, the desired turning radius must first be determined based on the robot's characteristics and operational requirements. Then, the steering angles for the front wheels can be calculated using the following formulas:

![Test run steering terminal](ackermann_vehicle/images/ackermann_steering.jpg)




# Tested version: 
ROS Noetic running Ubuntu 20.04



ackermann_vehicle (updated with ROS Noetic)
=================

ROS packages for simulating a vehicle with Ackermann steering

# This package is for developers only. 

## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/oguzhanbzglu/SergeantBot.git
sudo apt install ros-noetic-ackermann-msgs
cd ~/catkin_ws
catkin_make
```


## Running 
`roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch`

## To test the steering command: 

```
#go to terminal to this python directory:
#/ackermann_vehicle/ackermann_vehicle_navigation/scripts/
# /ackermann_vehicle/ackermann_vehicle_navigation/scripts$
# Run the following commands:
./cmd_vel_to_ackermann_drive.py
```

This python file will wait for /cmd_vel inputs. You can test it with another terminal. For example:

```
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}'
```

![Test run steering terminal](ackermann_vehicle/images/ackermann_steering.jpg)

# Video - Please click on the image
[![Watch the video](https://img.youtube.com/vi/nZZEMrxxz2o/maxresdefault.jpg)](https://youtu.be/nZZEMrxxz2o)
