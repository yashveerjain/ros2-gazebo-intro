import os

from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable,  ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition



def generate_launch_description():
    return LaunchDescription([
    SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),
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
        )])