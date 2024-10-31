#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to your package
    autonomous_control_package = 'autonomous_control_PID'
    autonomous_control_executable = 'autonomous_control_PID_node'

    # Define the gps package and executable
    gps_read_package = 'gps_read_cpp'
    gps_read_executable = 'gps_read'
    
    # Motor Control
    motor_control_package = 'motor_control'
    motor_control_executable = 'motor_control_node'

    # Define the joystick_reader package and executable
    joystick_reader_package = 'joystick_reader'
    joystick_reader_executable = 'joystick_reader_node'

    # Define the imu_data_node package and executable
    mag_data_package = 'dfr_10_dof_pkg'
    mag_data_executable = 'magneometer_node'
    
    # Define the imu_data_node package and executable
    imu_data_package = 'dfr_10_dof_pkg'
    imu_data_executable = 'imu_data_node'

    # Define the imu_data_node package and executable
    environment_data_package = 'dfr_10_dof_pkg'
    environment_data_executable = 'environment_data_node'

    # Define the imu_data_node package and executable
    data_logger_package = 'data_logger_pkg'
    data_logger_executable = 'data_logger_node'

     # Create the Node action for magneometer_node
    magneometer_node = Node(
        package=mag_data_package,
        executable=mag_data_executable,
        name='magneometer_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Create the Node action for imu_data_node
    imu_data_node = Node(
        package=imu_data_package,
        executable=imu_data_executable,
        name='imu_data_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Motor control
    motor_control_node = Node(
        package=motor_control_package,
        executable=motor_control_executable,
        name='motor_control_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Create the Node action for remote_control_node
    autonomous_control_node = Node(
        package=autonomous_control_package,
        executable=autonomous_control_executable,
        name='autonomous_control_node',
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

    # Create the Node action for remote_control_node
    gps_node = Node(
        package=gps_read_package,
        executable=gps_read_executable,
        name='gps_read_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Create the Node action for environmental monitoring node
    environment_data_node = Node(
        package=environment_data_package,
        executable=environment_data_executable,
        name='environment_data_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )

    # Create the Node action for data logger
    data_logger = Node(
        package=data_logger_package,
        executable=data_logger_executable,
        name='data_logger_node',
        output='screen',
        parameters=[
            # You can add parameters here if your node requires them
        ]
    )


    # Return the LaunchDescription with both nodes
    return LaunchDescription([
        autonomous_control_node,
        motor_control_node,
        data_logger,
        joystick_reader_node,
        imu_data_node,
        magneometer_node,
        gps_node,
        environment_data_node
    ])
