from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    markers_arg = DeclareLaunchArgument(
        'markers', default_value='false',
        description='Launch marker-based localization')

    pkg = FindPackageShare('sirius_navigation')

    map_sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'map_sensor_fusion.launch.py'])
        ),
    )

    # Static publisher that anchors map→odom while map EKF runs
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_sensor_unfusion',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        markers_arg,
        static_tf_node,
        map_sensor_fusion,
    ])
