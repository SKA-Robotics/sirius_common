"""
Launch ros2_control for Gazebo simulation.

The GazeboSimSystem hardware plugin is used — the controller_manager is
started by gz_ros2_control via the Gazebo plugin declared in the URDF,
so here we only spawn the controllers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    manipulator_arg = DeclareLaunchArgument(
        'manipulator',
        default_value='none',
        choices=['none', '5DOF', '6DOF'],
        description='Which manipulator variant is simulated')

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_base = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sirius_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_manip = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manip_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('manipulator'), "' != 'none'"])
        ),
    )

    # Give Gazebo + gz_ros2_control time to initialise
    delayed = TimerAction(
        period=5.0,
        actions=[spawn_jsb, spawn_base, spawn_manip],
    )

    return LaunchDescription([
        manipulator_arg,
        delayed,
    ])
