from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from math import sqrt


def generate_launch_description():
    imu_name_arg = DeclareLaunchArgument(
        'imu_name', default_value='front_camera',
        description='Prefix of the IMU sensor topics')

    imu_name = LaunchConfiguration('imu_name')

    imu_fusion_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_fusion',
        output='screen',
        parameters=[{
            'use_mag': False,
            'orientation_stddev': sqrt(0.0004079370468171413),
            'world_frame': 'enu',
            'publish_tf': False,
            'remove_gravity_vector': True,
        }],
        remappings=[
            ('imu/data_raw', ['/', imu_name, '/imu/data']),
            ('imu/mag', ['/', imu_name, '/imu/mag']),
            ('imu/data', ['/', imu_name, '/imu_compensated/data']),
        ],
    )

    imu_transformer_node = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        name='imu_data_transformer',
        output='screen',
        parameters=[{'target_frame': 'base_link'}],
        remappings=[
            ('imu_in/data', ['/', imu_name, '/imu_compensated/data']),
            ('imu_out/data', '/imu_compensated/data'),
        ],
    )

    return LaunchDescription([
        imu_name_arg,
        imu_fusion_node,
        imu_transformer_node,
    ])
