from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dxl_wsl',
            executable='pub',
            name='node_dxlpub',
            output='screen',
            prefix='xterm -fa "Monospace" -fs 11 -e',
        ),
        Node(
            package='camera_ros2',
            executable='sub',
            name='camsub_wsl',
            output='screen',
        ),
    ])
