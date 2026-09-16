from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sirius_navigation')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'params_file': PathJoinSubstitution([pkg, 'config', 'move_base.yaml']),
        }.items(),
    )

    simple_goal_relay = Node(
        package='sirius_navigation',
        executable='simple_goal_relay.py',
        name='simple_goal_relay',
        output='screen',
    )

    return LaunchDescription([
        nav2_bringup,
        simple_goal_relay,
    ])
