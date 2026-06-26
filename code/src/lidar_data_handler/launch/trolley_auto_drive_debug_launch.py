#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    handler_node = Node(
        package='lidar_data_handler',
        executable='lidar_data_handler',
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='DISPLAY', value=':0'),
        SetEnvironmentVariable(name='LANG', value='C.UTF-8'),
        SetEnvironmentVariable(name='LC_ALL', value='C.UTF-8'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='TROLLEY_DEBUG_MODE', value='1'),
        handler_node,
    ])
