import launch
from launch.substitutions import LaunchConfiguration

import launch_ros
import os

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(
        package='sirius_description').find('sirius_description')
    rvizConfigPath = os.path.join(pkgPath, 'config/config.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    return launch.LaunchDescription([rviz_node])
    