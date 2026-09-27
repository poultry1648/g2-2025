"""Display the g2 URDF in RViz2.

Runs robot_state_publisher on the URDF and opens RViz2 with a config that
already contains a RobotModel display and the chassis as the fixed frame.

Usage:
  ros2 launch g2_description display.launch.py
  ros2 launch g2_description display.launch.py model:=/path/to/other.urdf
  ros2 launch g2_description display.launch.py frame:=shooter_body
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    g2_share = get_package_share_directory('g2_description')

    default_model = os.path.join(g2_share, 'urdf', 'g2.urdf')
    default_rviz = os.path.join(g2_share, 'config', 'display.rviz')

    model = LaunchConfiguration('model')
    frame = LaunchConfiguration('frame')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description = ParameterValue(
        Command(['cat ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the URDF file to display.',
        ),
        DeclareLaunchArgument(
            'frame',
            default_value='chassis',
            description='Fixed frame for RViz2.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Absolute path to the RViz2 config file.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config, '-f', frame],
            output='screen',
        ),
    ])
