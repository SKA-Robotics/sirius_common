from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'frequency': 20.0,
            'delay': 0.0,
            'transform_timeout': 0.05,
            'zero_altitude': True,
            'broadcast_cartesian_transform': True,
            'broadcast_cartesian_transform_as_parent_frame': True,
            'publish_filtered_gps': True,
            'wait_for_datum': True,
            'datum': [50.0662169, 19.913204399999998, -1.855997665],
        }],
        remappings=[
            ('odometry/filtered', '/pose'),
            ('odometry/gps', '/gps/pose'),
        ],
    )

    return LaunchDescription([navsat_node])
