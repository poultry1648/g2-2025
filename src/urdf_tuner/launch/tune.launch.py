"""
Launch the interactive URDF tuner with RViz2.

Runs the slider panel (which publishes the live TF tree and joint axes)
alongside RViz2 configured with a RobotModel, TF and MarkerArray display.

Usage:
  ros2 launch g2_urdf_tuner tune.launch.py
  ros2 launch g2_urdf_tuner tune.launch.py urdf:=/abs/robot.urdf save:=/abs/out.urdf

The default URDF is resolved from ``src/g2_description/urdf/g2.urdf`` relative
to the current working directory so edits save straight back into the source
tree; otherwise the installed share copy is used.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_urdf():
    candidate = os.path.join(os.getcwd(), 'src', 'g2_description', 'urdf', 'g2.urdf')
    if os.path.isfile(candidate):
        return candidate
    return os.path.join(get_package_share_directory('g2_description'), 'urdf', 'g2.urdf')


def generate_launch_description():
    """Build the launch description for the tuner and RViz2."""
    default_urdf = _default_urdf()
    default_rviz = os.path.join(
        get_package_share_directory('g2_urdf_tuner'), 'config', 'tune.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            default_value=default_urdf,
            description='Absolute path to the URDF file to tune.',
        ),
        DeclareLaunchArgument(
            'save',
            default_value=default_urdf,
            description='Absolute path the tuned URDF is written to.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Absolute path to the RViz2 config file.',
        ),
        Node(
            package='g2_urdf_tuner',
            executable='urdf_tuner_gui',
            parameters=[{
                'urdf_path': LaunchConfiguration('urdf'),
                'save_path': LaunchConfiguration('save'),
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
