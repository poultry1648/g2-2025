from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock as the ROS time source.',
        ),
        Node(
            package='kitbot_application',
            executable='diff_drive',
            name='diff_drive',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wheel_radius': 0.0762,
                'track_width': 0.468,
                'left_direction': -1.0,
                'right_direction': 1.0,
            }],
            output='screen',
        ),
    ])
