from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('sirius_tty_sensors'),
        'config',
        'sensors.yaml'
    ])

    return LaunchDescription([
        Node(
            package='sirius_tty_sensors',
            executable='sensors',
            name='sensors',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])