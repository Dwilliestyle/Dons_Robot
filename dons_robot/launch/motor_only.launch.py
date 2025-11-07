from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dons_robot',
            executable='esp32_bridge',
            name='esp32_bridge',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baud_rate': 115200,
                'timeout': 0.5,
                'watchdog_timeout': 0.5
            }],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.15,
                'autorepeat_rate': 20.0
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 2,
                'scale_linear.x': 0.5,
                'scale_angular.yaw': 5.0,
                'enable_button': 4,
                'enable_turbo_button': 5,
                'require_enable_button': True
            }]
        )
    ])