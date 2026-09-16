"""
MoveIt2 + Gazebo Harmonic simulation launch.

Starts Gazebo, ros2_control (sim), and move_group together.

Usage:
  ros2 launch sirius_moveit_config moveit_gazebo.launch.py manipulator:=5DOF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                   PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manipulator_arg = DeclareLaunchArgument(
        'manipulator',
        default_value='6DOF',
        choices=['5DOF', '6DOF'])

    manipulator = LaunchConfiguration('manipulator')

    # ---- Gazebo simulation ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sirius_gazebo'),
            'launch', 'sirius_gazebo.launch.py')),
        launch_arguments={'manipulator': manipulator}.items(),
    )

    # ---- Controller spawners (after Gazebo has started) ----
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sirius_moveit_config'),
            'launch', 'moveit.launch.py')),
        launch_arguments={
            'manipulator': manipulator,
            'use_real_hardware': 'false',
            'use_sim_time': 'true',
        }.items(),
    )

    # Delay MoveIt until Gazebo physics is running
    delayed_moveit = TimerAction(period=8.0, actions=[moveit_launch])

    return LaunchDescription([
        manipulator_arg,
        gazebo_launch,
        delayed_moveit,
    ])
