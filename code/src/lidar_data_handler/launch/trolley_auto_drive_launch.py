#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    share_dir = get_package_share_directory('olei_driver')
    front_parameter_file = LaunchConfiguration('front_params_file')
    back_parameter_file = LaunchConfiguration('back_params_file')

    front_params_declare = DeclareLaunchArgument('front_params_file',
                                           default_value=os.path.join(
                                               share_dir, 'config', 'front_lidar.yaml'),
                                           description='Path to the front lidar ROS2 parameters file.')
    back_params_declare = DeclareLaunchArgument('back_params_file',
                                           default_value=os.path.join(
                                               share_dir, 'config', 'back_lidar.yaml'),
                                           description='Path to the back lidar ROS2 parameters file.')

    front_lidar_driver_node = Node(package='olei_driver',
                                executable='ros_main',
                                name='front_lidar_node',
                                output='screen',
                                parameters=[front_parameter_file],
                                )
    
    back_lidar_driver_node = Node(package='olei_driver',
                                executable='ros_main',
                                name='back_lidar_node',
                                output='screen',
                                parameters=[back_parameter_file],
                                )
    
    left_linear_sensor_node = Node(package='linear_displacement_sensor',
                                executable='left_sensor',
                                output='screen',
                                )
    
    right_linear_sensor_node = Node(package='linear_displacement_sensor',
                                executable='right_sensor',
                                output='screen',
                                )
    
    handler_node = Node(package='lidar_data_handler',
                        executable='lidar_data_handler',
                        output='screen',
                        )

    return LaunchDescription([
        left_linear_sensor_node,
        right_linear_sensor_node,
        front_params_declare,
        back_params_declare,
        front_lidar_driver_node,
        back_lidar_driver_node,
        TimerAction(
            period=5.0,  # 5秒后启动主程序节点
            actions=[handler_node]
        )
    ])
