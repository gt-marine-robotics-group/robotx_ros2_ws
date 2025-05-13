#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unstatic', 
            executable='drone_unstatic',
            name='drone_unstatic',
            output='screen',
            emulate_tty=True,
            parameters=[{'udp': 'udp:127.0.0.1:14550'}],  # Assuming these are parameters your drone_node needs
        ),
        Node(
            package='unstatic',
            executable='camera_unstatic',
            name='camera_unstatic',
            output='screen',
            emulate_tty=True,
            # No parameters are passed in this example, add if needed
        ),
        Node(
            package='unstatic',
            executable='mission_unstatic',
            name='mission_unstatic',
            output='screen',
            emulate_tty=True,
            # No parameters are passed in this example, add if needed
        ),
    ])
