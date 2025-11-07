from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory('dons_robot')
    
    # Camera config file
    camera_config = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    
    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[camera_config],
        output='screen'
    )
    
    # Web Video Server (for browser viewing)
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )
    
    return LaunchDescription([
        usb_cam_node,
        web_video_server
    ])