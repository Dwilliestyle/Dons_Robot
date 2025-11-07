import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('dons_robot')
    urdf_file = os.path.join(package_dir, 'urdf', 'dons_robot_gazebo.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'dons_robot']
        )
    ])