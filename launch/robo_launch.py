import os

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable,  ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition



def generate_launch_description():
    return LaunchDescription([
    SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),
    DeclareLaunchArgument(
            'enable_recording',
            default_value='True'
        ),
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch/empty_world.launch.py']))  
        ),
    Node(
            package='working_with_gazebo',
            executable='walker',
            name='walker',
            remappings=[
                ('/walker/cmd_vel','/cmd_vel'),
            ]
        ),
    ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('enable_recording')
                ])
            ),
            cmd=[[
                'ros2 bag record -o bag_output ',
                '/cmd_vel ',
                '/scan ',
                '/imu ',
                '/odom '
            ]],
            shell=True
        ),])