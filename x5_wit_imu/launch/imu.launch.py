#!/usr/bin/env python3
# 亚博 10 轴 IMU（WitMotion 协议）C++ 驱动；与 imu_ros2_device/launch/imu.launch.py 的 Python 驱动可互换。
# /dev/imu 还没建好时：ros2 launch x5_wit_imu imu.launch.py port:=/dev/ttyUSB2
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
            package='x5_wit_imu',
            executable='wit_imu_node',
            name='imu',
            output='screen',
            # 转接头不在时不要反复起进程、写日志
            additional_env={'WHEELCHAIR_SERIAL_PORT': LaunchConfiguration('port')},
            prefix='bash -c \'while [ ! -c "$WHEELCHAIR_SERIAL_PORT" ]; do sleep 1; done; exec "$@"\' --',
            parameters=[{'port': LaunchConfiguration('port'),
                         'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int)}],
            respawn=True, respawn_delay=3.0,
        )
    ])
