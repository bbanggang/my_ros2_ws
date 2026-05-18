from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_int_rclpy_pkg',
            executable='int_pub',
            name='int_publisher',
            output='screen',
        ),
        Node(
            package='my_int_rclpy_pkg',
            executable='int_sub',
            name='int_subscriber',
            output='screen',
        ),
    ])
