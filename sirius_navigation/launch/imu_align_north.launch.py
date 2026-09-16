from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declination_arg = DeclareLaunchArgument(
        'declination', default_value='0.0',
        description='Magnetic declination in degrees')

    return LaunchDescription([
        declination_arg,
        Node(
            package='sirius_navigation',
            executable='imu_align_north.py',
            name='imu_align_north',
            output='screen',
            parameters=[{
                'declination': LaunchConfiguration('declination'),
            }],
        ),
    ])
