#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ros2_lidar')
    front_parameter_file = LaunchConfiguration('front_params_file')
    back_parameter_file = LaunchConfiguration('back_params_file')
    node_name = 'lidar_driver'

    front_params_declare = DeclareLaunchArgument('front_params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ole2dv2_front.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
    back_params_declare = DeclareLaunchArgument('back_params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ole2dv2_back.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    front_lidar_driver_node = LifecycleNode(package='ros2_lidar',
                                executable='lidar_driver',
                                name=node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[front_parameter_file],
                                namespace='/front_lidar',
                                )
    
    back_lidar_driver_node = LifecycleNode(package='ros2_lidar',
                                executable='lidar_driver',
                                name=node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[back_parameter_file],
                                namespace='/back_lidar',
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

    front_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(front_lidar_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    front_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=front_lidar_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] front lidar driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(front_lidar_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    back_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(back_lidar_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    back_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=back_lidar_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] front lidar driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(back_lidar_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # TODO make lifecycle transition to shutdown before SIGINT
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name=node_name),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] lidar driver node is exiting."),
            ],
        )
    )

    return LaunchDescription([
        left_linear_sensor_node,
        right_linear_sensor_node,
        front_params_declare,
        back_params_declare,
        front_lidar_driver_node,
        back_lidar_driver_node,
        front_activate_event,
        front_configure_event,
        back_activate_event,
        back_configure_event,
        shutdown_event,
        TimerAction(
            period=3.0,  # 3秒后启动主程序节点
            actions=[handler_node]
        )
    ])
