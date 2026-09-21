import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('kitbot_description'),
        'launch',
        'gazebo.launch.py',
    )
    application_launch = os.path.join(
        get_package_share_directory('kitbot_application'),
        'launch',
        'application.launch.py',
    )

    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bridge_topics = [
        '/left_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/right_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/roller_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]

    return LaunchDescription([
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock as the ROS time source.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world,
                'x': spawn_x,
                'y': spawn_y,
                'z': spawn_z,
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=bridge_topics,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(application_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
