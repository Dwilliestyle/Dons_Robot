#!/usr/bin/env python3
"""
Launch file for Don's RaspRover robot teleop
Starts ESP32 bridge + joystick control + keyboard backup + camera
Uses robot_params.yaml for configuration
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('dons_robot')
    
    # Load parameter files
    config_file = os.path.join(package_dir, 'config', 'robot_params.yaml')
    camera_config = os.path.join(package_dir, 'config', 'camera_params.yaml')
    
    # Get URDF file for robot description
    urdf_file = os.path.join(package_dir, 'urdf', 'dons_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Declare launch arguments
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Whether to use joystick control'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to start the LIDAR'
    )
    
    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard',
        default_value='false',
        description='Whether to use keyboard control'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to start the camera'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for robot topics'
    )
    
    # Get launch arguments
    use_joystick = LaunchConfiguration('use_joystick')
    use_keyboard = LaunchConfiguration('use_keyboard')
    use_camera = LaunchConfiguration('use_camera')
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_lidar = LaunchConfiguration('use_lidar')
    
    # ESP32 Bridge Node - Core robot interface
    esp32_bridge_node = Node(
        package='dons_robot',
        executable='esp32_bridge',
        name='esp32_bridge',
        namespace=robot_namespace,
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    # Robot State Publisher - Publishes robot description/transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        parameters=[
            config_file,
            {'robot_description': robot_description_content}
        ],
        output='screen'
    )
    
    # Joy Node - Reads joystick/gamepad input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=robot_namespace,
        parameters=[config_file],
        condition=IfCondition(use_joystick),
        output='screen'
    )
    
    # Teleop Twist Joy - Converts joystick to cmd_vel
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        namespace=robot_namespace,
        parameters=[config_file],
        condition=IfCondition(use_joystick),
        output='screen'
    )
    
    # Teleop Twist Keyboard - Keyboard control backup
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        namespace=robot_namespace,
        prefix='xterm -e',
        condition=IfCondition(use_keyboard),
        output='screen'
    )
    
    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=robot_namespace,
        parameters=[camera_config],
        condition=IfCondition(use_camera),
        output='screen'
    )
    
    # Web Video Server (for browser viewing)
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        namespace=robot_namespace,
        condition=IfCondition(use_camera),
        output='screen'
    )

    # LIDAR Node
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        namespace=robot_namespace,
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'base_laser',
            'port_name': '/dev/ttyACM0',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0
        }],
        condition=IfCondition(use_lidar),
        output='screen'
    )

    # LIDAR Transform - base_link to base_laser
    lidar_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        namespace=robot_namespace,
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser'],
        condition=IfCondition(use_lidar),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_joystick_arg,
        use_keyboard_arg,
        use_camera_arg,
        use_lidar_arg,
        robot_namespace_arg,
        
        # Core nodes
        esp32_bridge_node,
        robot_state_publisher_node,
        
        # Teleop nodes (conditional)
        joy_node,
        teleop_twist_joy_node,
        teleop_keyboard_node,
        
        # Camera nodes (conditional)
        usb_cam_node,
        web_video_server,

        # LIDAR nodes (conditional)
        lidar_node,
        lidar_transform_node,
    ])
