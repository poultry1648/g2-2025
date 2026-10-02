import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('motor_node')
    default_config = os.path.join(share, 'config', 'motors.yaml')
    bridge_config = os.path.join(share, 'config', 'g2_bridge.yaml')
    bridge_launch = os.path.join(
        get_package_share_directory('ros_gz_bridge'),
        'launch', 'ros_gz_bridge.launch.py')

    config_file = LaunchConfiguration('config_file')
    transport = LaunchConfiguration('transport')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_bridge = LaunchConfiguration('start_bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML file describing the motors.',
        ),
        DeclareLaunchArgument(
            'transport',
            default_value='',
            description='Motor backend: sim or talonfx. Empty uses the config file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock as the ROS time source.',
        ),
        DeclareLaunchArgument(
            'start_bridge',
            default_value='true',
            description='Bridge the effort and joint-state topics to Gazebo.',
        ),
        Node(
            package='motor_node',
            executable='motor_node',
            name='motor_node',
            parameters=[{
                'config_file': config_file,
                'transport': transport,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={
                'config_file': bridge_config,
                'bridge_name': 'motor_bridge',
            }.items(),
            condition=IfCondition(start_bridge),
        ),
    ])
