# Working with Gazebo
This repo contains the introductory implementation of turtlebot3 in gazebo using ros2
* Name : Yashveer Jain
---
## Run
### To run with single launch file:
```
ros2 launch working_with_gazebo/launch/robo_launch.py
```

### To run turtlebot and custom code separatly
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 run working_with_gazebo walker --ros-args -r /walker/cmd_vel:=/cmd_vel
```
### Output
* gazebo simulation window will open
* Turtle bot will start moving forward
* Note: To check the bot is avoiding obstacles, place the objects in front of the robot when it, and when the object distance within `1 meter` distance, robot will turn to left, until the obstacle is totally avoided.
---
## Dependencies
* ros2 humble
* turtlebot3, installation can be found [here](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)
* geometry_msgs
* sensor_msgs
