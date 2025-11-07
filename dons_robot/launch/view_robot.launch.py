#!/usr/bin/env python3

"""
Launch file for viewing Don's RaspRover robot in RViz
Includes robot description, joint state publisher, and pre-configured RViz
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    package_dir = get_package_share_directory('dons_robot')
    
    # Get URDF file
    urdf_file = os.path.join(package_dir, 'urdf', 'dons_robot.urdf')
    
    # Get RViz config file
    rviz_config_file = os.path.join(package_dir, 'rviz', 'robot_view.rviz')
    
    # Read robot description
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher - publishes robot description and transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        # Joint State Publisher GUI - provides sliders for wheel movement
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz2 - visualization with saved configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],  # Load saved RViz config
            output='screen'
        )
    ])