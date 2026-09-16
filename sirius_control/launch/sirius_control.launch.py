"""
Launch ros2_control for the Sirius rover (real hardware).

Loads the controller manager with SiriusWheelsHW and (optionally)
SiriusManipHW, then activates all controllers defined in controllers.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                             TimerAction)
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manipulator_arg = DeclareLaunchArgument(
        'manipulator',
        default_value='none',
        choices=['none', '5DOF', '6DOF'],
        description='Which manipulator variant is mounted')

    description_pkg = get_package_share_directory('sirius_description')
    xacro_path = os.path.join(description_pkg, 'robots', 'sirius.urdf.xacro')
    controllers_yaml = os.path.join(description_pkg, 'config', 'controllers.yaml')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
            ' manipulator:=', LaunchConfiguration('manipulator'),
            ' use_real_hardware:=true',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # ---- spawn controllers after controller_manager starts ----

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    spawn_base = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sirius_base_controller', '--controller-manager', '/controller_manager'],
    )

    spawn_manip = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manip_controller', '--controller-manager', '/controller_manager'],
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(
            __import__('launch.substitutions', fromlist=['PythonExpression']).PythonExpression(
                ["'", LaunchConfiguration('manipulator'), "' != 'none'"]
            )
        ),
    )

    # Use a timer to give controller_manager a moment to load plugins
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[spawn_jsb, spawn_base, spawn_manip],
    )

    return LaunchDescription([
        manipulator_arg,
        robot_state_publisher,
        controller_manager,
        delayed_spawners,
    ])
