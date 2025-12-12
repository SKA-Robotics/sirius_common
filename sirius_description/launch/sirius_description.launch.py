import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'sirius_description'
    
    xacro_file_name = 'sirius.urdf.xacro' 
    
    pkg_path = get_package_share_directory(package_name)
    xacro_path = os.path.join(pkg_path, 'robots', xacro_file_name)

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])