#!/usr/bin/env python3
"""
SLAM Mapping launch file for Don's RaspRover
Starts SLAM Toolbox for mapping with auto-activation
"""
import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('dons_robot')
    
    # SLAM Toolbox config
    slam_config = os.path.join(package_dir, 'config', 'mapper_params_online_async.yaml')
    
    # SLAM Toolbox Lifecycle Node
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config]
    )
    
    # Automatically configure the node when it starts
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # When configure succeeds, activate the node
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    return LaunchDescription([
        slam_toolbox_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=slam_toolbox_node,
                on_start=[configure_event],
            )
        ),
        activate_event,
    ])