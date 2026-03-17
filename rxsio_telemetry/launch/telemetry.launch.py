from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('rxsio_telemetry'),
        'config',
        'telemetry.yaml'
    ])

    return LaunchDescription([
        Node(
            package='rxsio_telemetry',
            executable='telemetry',
            name='telemetry',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])