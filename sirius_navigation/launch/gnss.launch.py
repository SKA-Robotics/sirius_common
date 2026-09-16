from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps',
        output='screen',
        parameters=[{
            'device': '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',
            'frame_id': 'gnss',
            'config_on_startup': False,
        }],
        remappings=[
            ('fix', '/gps/fix'),
            ('fix_velocity', '/gps/fix_velocity'),
        ],
    )

    return LaunchDescription([ublox_node])
