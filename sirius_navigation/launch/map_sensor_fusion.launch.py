from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('sirius_navigation'),
        'config', 'map_sensor_fusion.yaml',
    ])

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='map_sensor_fusion',
            output='screen',
            parameters=[config],
            remappings=[('odometry/filtered', '/pose')],
        ),
    ])
