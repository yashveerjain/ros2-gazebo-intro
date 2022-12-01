# Working with Gazebo
This repo contains the introductory implementation of turtlebot3 in gazebo using ros2
* Name : Yashveer Jain
## Overview
The robot using lidar scan detect the distance from the obstacle in 360 deg view,
and if the obstacle lie in front (between -20 deg to 20 deg angle) of the robot within 1 meter distance, the robot will turn to left until it avoided the obstacles.
---
## Installation
* Inside the ros2_ws (ros 2 working space) dir:
```
colcon build --packages-select working_with_gazebo
. install/setup.bash
```
---
## Run
### To run with single launch file:
* Inside the ros2_ws/src dir:
```
ros2 launch working_with_gazebo/launch/robo_launch.py  enable_recording:=False
```

### To run turtlebot and custom code separatly
* In one terminal run :
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* On another terminal run:
```
ros2 run working_with_gazebo walker --ros-args -r /walker/cmd_vel:=/cmd_vel
```

### Output
* gazebo simulation window will open
* Turtle bot will start moving forward
* Note: To check the bot is avoiding obstacles, place the objects in front of the robot when it, and when the object distance within `1 meter` distance, robot will turn to left, until the obstacle is totally avoided.
---
## Run the ros2 bag recordings
* In one terminal run:
```
ros2 bag play working_with_gazebo/results/bag_output/
```

## Dependencies
* ros2 humble
* turtlebot3, installation can be found [here](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)
* geometry_msgs
* sensor_msgs

## Results
* Ros2 bag recorded [file](results/bag_output/) and is sqlite format.
* cppcheck output is [here](results/cppcheck-output.txt).
* cpplint output is [here](results/cpplint-output.txt).
