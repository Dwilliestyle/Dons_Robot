from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    dons_robot_dir = get_package_share_directory('dons_robot')
    
    return LaunchDescription([
        # Include your normal robot teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dons_robot_dir, 'launch', 'robot_teleop.launch.py')
            )
        ),
        
        # Static transform for odometry
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'use_scan_matching': True,
                'use_scan_barycenter': True
            }],
            output='screen'
        )
    ])