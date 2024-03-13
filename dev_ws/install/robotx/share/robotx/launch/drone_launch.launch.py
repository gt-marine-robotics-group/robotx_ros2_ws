#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotx', 
            executable='drone_node',
            name='drone_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'udp': 'udp:127.0.0.1:14550'}],  # Assuming these are parameters your drone_node needs
        ),
        Node(
            package='robotx',
            executable='camera_node',
            name='camera_node',
            output='screen',
            emulate_tty=True,
            # No parameters are passed in this example, add if needed
        ),
        Node(
            package='robotx',
            executable='mission_node',
            name='mission_node',
            output='screen',
            emulate_tty=True,
            # No parameters are passed in this example, add if needed
        ),
    ])
