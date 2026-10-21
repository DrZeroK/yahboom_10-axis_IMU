#!/usr/bin/env python3
# Yahboom 10-axis IMU (WitMotion protocol). Factory serial: 9600 baud, 10 Hz.
# Before /dev/imu exists: ros2 launch imu_ros2_device imu.launch.py port:=/dev/ttyUSB2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/imu'),
        DeclareLaunchArgument('baud', default_value='9600'),
        Node(
            package='imu_ros2_device',
            executable='wit_imu_driver',
            name='imu',
            output='screen',
            # Avoid repeated process/log allocation while the adapter is absent.
            additional_env={'WHEELCHAIR_SERIAL_PORT': LaunchConfiguration('port')},
            prefix='bash -c \'while [ ! -c "$WHEELCHAIR_SERIAL_PORT" ]; do sleep 1; done; exec "$@"\' --',
            parameters=[{'port': LaunchConfiguration('port'),
                         'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int)}],
            respawn=True, respawn_delay=3.0,
        )
    ])
