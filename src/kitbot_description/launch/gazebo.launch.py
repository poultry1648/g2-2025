import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    kitbot_share = get_package_share_directory('kitbot_description')
    g2_gazebo_share = get_package_share_directory('g2_gazebo')

    urdf_file = os.path.join(kitbot_share, 'urdf', 'kb_25000.urdf')
    world_file = os.path.join(g2_gazebo_share, 'models', 'reefscape.world')

    gz_args = LaunchConfiguration('gz_args')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')

    resource_path = os.pathsep.join([
        os.path.dirname(kitbot_share),
        os.path.join(g2_gazebo_share, 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r ' + world_file,
            description='Arguments for Gazebo Sim. Use "-s -r ' + world_file +
                        '" to run headless.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='reefscape',
            description='Name of the world the robot is spawned into.',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='-6.0',
            description='X (m) the robot is spawned at.',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Y (m) the robot is spawned at.',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.5',
            description='Height (m) the robot is dropped from.',
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', world,
                    '-name', 'kitbot',
                    '-file', urdf_file,
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                ],
                output='screen',
            ),
        ]),
    ])
