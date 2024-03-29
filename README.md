# SergeantBot

SergeantBot, I am working on an autonomous controlled vehicle project for my engineering department thesis using the ROS (Robot Operating System) software. The robot uses the Ackermann Steering method for its motion. Ackermann Steering is a type of steering mechanism commonly used in vehicles and mobile robots, including autonomous mobile robots. It is based on the principle that the inner wheel turns at a smaller angle than the outer wheel during a turn, resulting in both wheels following circular paths with different radius. This method provides stability and facilitates control of the vehicle or robot by preventing slipping. The main formulation of the Ackermann steering involves calculating the steering angles for the front wheels based on the desired turning radius and the vehicle's geometry. The steering angles are typically expressed as a function of the wheelbase (the distance between the front and rear axles) and the track width (the distance between the centers of the front wheels).

To apply Ackermann steering to an autonomous mobile robot, the desired turning radius must first be determined based on the robot's characteristics and operational requirements. Then, the steering angles for the front wheels can be calculated using the following formulas:

![Ackermann Steering](ackermann_vehicle/images/ackermann_steering.jpeg)
![Formula](ackermann_vehicle/images/formula.jpeg)

## Software
- Ubuntu 20.04
- ROS | Noetic
- Python 3

## Hardware
- Raspberry Pi Model 3B+
- YD LIDAR 2X
- MPU 6050 Gyro Sensor
- 12V DC Motor
- Servo Motor
- PCA 9685 Servo Driver
- LM393 Motor Speed Sensor (I use that as an encoder with encoder wheel)
- Hobbywing Quickrun 1060 ESC
- 5200mAh Battery


## Robot

![Sergeantbot](ackermann_vehicle/images/sergeantbot.png)
![Sergeantbot](ackermann_vehicle/images/sergeantbot2.png)


## Path Follower 

The path was given as a trajectory data.

![sergeant_gazebo](ackermann_vehicle/images/sergeant_gazebo.png)


![Husky_path](ackermann_vehicle/images/husky_gazebo.png)
![Sergeant_path](ackermann_vehicle/images/path_gazebo.png)



# TESTS

ROS packages for simulating a vehicle with Ackermann steering


### Installation
```
cd ~/catkin_ws/src
git clone https://github.com/oguzhanbzglu/SergeantBot.git
sudo apt install ros-noetic-ackermann-msgs
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```


### Running 
`roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch`

#### To test the steering command: 

```
#go to terminal to this python directory:
#/ackermann_vehicle/ackermann_vehicle_navigation/scripts/
# Run the following commands:
./cmd_vel_to_ackermann_drive.py
```

This python file will wait for /cmd_vel inputs. You can test it with another terminal. For example:

```
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}'
```

## Run the path follower

```
#run each command in different terminals (I should create a launch file :D )
#when you run the .launch file if the path is different you are able to change the world based on the name in /ackermann_vehicle_gazebo/worlds
$ rosrun ackermann_vehicle_navigation tf_odom_publisher.py
$ rosrun ackermann_vehicle_navigation cmd_vel_to_ackermann_drive.py
$ rosrun ackermann_vehicle_navigation path_publisher.py
```
```
$ rosrun ackermann_vehicle_navigation path_follower.py
```
### Testing robot in Gazebo (Click the image to see video)
[![Watch the video](ackermann_vehicle/images/followers_path.png)](https://youtu.be/u6-OWd0uj58)

### Steering test of the robot (Click the image to see video)

### Steering Test

```
     |W|
|A|  |S|  |D|


run the roscore in a different terminal then:
$ rosrun idurobot control_robot.py
```
[![Watch the video](ackermann_vehicle/images/the_robot.jpeg)](https://youtu.be/c5ZcQJwzZ2Y)

[![Terminal print](ackermann_vehicle/images/terminal.png)]


