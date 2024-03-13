#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotx',  # Name of the package containing the node
            executable='drone_node',  # Name of the executable
            output='screen',  # Optional: Direct node output to the console
            # You can add more parameters or configurations here
        ),
        Node(
            package='robotx',  # Name of the package containing the node
            executable='mission_unmoving_landing',  # Name of the executable
            output='screen',  # Optional: Direct node output to the console
            # You can add more parameters or configurations here
        ),
    ])