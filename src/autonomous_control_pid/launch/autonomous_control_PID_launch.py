#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to your package
    remote_control_package = 'autonomous_control_PID'
    remote_control_executable = 'autonomous_control_PID_node'

    # Define the joystick_reader package and executable
    joystick_reader_package = 'joystick_reader'
    joystick_reader_executable = 'joystick_reader_node'

    # Define the gps_read_cpp
    gps_package = 'gps_read_cpp'
    gps_read_executable = 'gps_read'

    # Define imu drivers

    # Define 

    # Create the Node action for remote_control_node
    remote_control_node = Node(
        package=remote_control_package,
        executable=remote_control_executable,
        name='remote_control_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Create the Node action for joystick_reader_node
    joystick_reader_node = Node(
        package=joystick_reader_package,
        executable=joystick_reader_executable,
        name='joystick_reader_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Return the LaunchDescription with both nodes
    return LaunchDescription([
        joystick_reader_node,
        remote_control_node
    ])
