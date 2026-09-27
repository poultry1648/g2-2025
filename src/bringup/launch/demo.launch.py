import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('g2_bringup'),
        'config',
        'demo_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='g2_bringup',
            executable='g2_talker',
            name='g2_talker',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='g2_bringup',
            executable='g2_listener',
            name='g2_listener',
            output='screen',
        ),
    ])
