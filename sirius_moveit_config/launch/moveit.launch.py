"""
MoveIt2 launch file for the Sirius rover manipulator.

Usage:
  ros2 launch sirius_moveit_config moveit.launch.py manipulator:=5DOF
  ros2 launch sirius_moveit_config moveit.launch.py manipulator:=6DOF
  ros2 launch sirius_moveit_config moveit.launch.py manipulator:=6DOF use_real_hardware:=true

Starts:
  - robot_state_publisher (with ros2_control tags)
  - ros2_control node (real) or controller spawners (sim)
  - move_group
  - RViz with MoveIt plugin
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                   PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    manipulator_arg = DeclareLaunchArgument(
        'manipulator',
        default_value='6DOF',
        choices=['5DOF', '6DOF'],
        description='Manipulator variant')

    use_real_hw_arg = DeclareLaunchArgument(
        'use_real_hardware',
        default_value='false',
        description='Use real hardware interfaces instead of simulation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    manipulator = LaunchConfiguration('manipulator')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- URDF / robot_description ----
    description_pkg = get_package_share_directory('sirius_description')
    xacro_path = os.path.join(description_pkg, 'robots', 'sirius.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
            ' manipulator:=', manipulator,
            ' use_real_hardware:=', use_real_hardware,
        ]),
        value_type=str,
    )

    # ---- SRDF (semantic description) ----
    moveit_pkg = get_package_share_directory('sirius_moveit_config')
    srdf_5dof = os.path.join(moveit_pkg, 'config', 'sirius_5dof.srdf')
    srdf_6dof = os.path.join(moveit_pkg, 'config', 'sirius_6dof.srdf')

    with open(srdf_5dof) as f:
        robot_description_semantic_5dof = f.read()
    with open(srdf_6dof) as f:
        robot_description_semantic_6dof = f.read()

    # MoveIt2 doesn't support substitutions in SRDF string yet — we pick at
    # python evaluation time. Launch args are evaluated before nodes start so
    # this works when manipulator is known at launch.
    #
    # NOTE: to fully support runtime switching, generate the SRDF via xacro too.

    # ---- MoveIt parameters shared by both SRDF variants ----
    # YAML configs must be loaded as Python dicts — passing file paths via
    # parameters=[] causes ROS2 to use --params-file which requires
    # ros__parameters: wrapper that MoveIt config files don't have.
    controllers_yaml = os.path.join(description_pkg, 'config', 'controllers.yaml')

    kinematics_cfg = _load_yaml(
        os.path.join(moveit_pkg, 'config', 'kinematics.yaml'))
    ompl_cfg = _load_yaml(
        os.path.join(moveit_pkg, 'config', 'ompl_planning.yaml'))
    joint_limits_cfg = _load_yaml(
        os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'))
    moveit_controllers_cfg = _load_yaml(
        os.path.join(moveit_pkg, 'config', 'moveit_controllers.yaml'))

    # MoveIt2 Jazzy uses named planning pipelines — wrap OMPL config in its namespace.
    ompl_pipeline_cfg = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_cfg,
    }

    move_group_common_params = [
        {'robot_description': robot_description},
        {'use_sim_time': use_sim_time},
        {'robot_description_kinematics': kinematics_cfg},
        {'robot_description_planning': joint_limits_cfg},
        ompl_pipeline_cfg,
        moveit_controllers_cfg,
        # Disable octomap — no 3D sensor configured; empty string avoids runtime_error.
        {'sensors': ['']},
        {'octomap_resolution': 0.0},
    ]

    # ---- robot_state_publisher (real hardware only; Gazebo launch owns it in sim) ----
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_real_hardware),
    )

    # ---- ros2_control (real hardware only) ----
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(use_real_hardware),
    )

    # ---- controller spawners (real hardware only; Gazebo launch owns them in sim) ----
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   '/controller_manager'],
        output='screen',
        condition=IfCondition(use_real_hardware),
    )

    spawn_base = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sirius_base_controller', '--controller-manager',
                   '/controller_manager'],
        output='screen',
        condition=IfCondition(use_real_hardware),
    )

    spawn_manip = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manip_controller', '--controller-manager',
                   '/controller_manager'],
        output='screen',
        condition=IfCondition(use_real_hardware),
    )

    # ---- move_group for 5DOF ----
    move_group_5dof = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=move_group_common_params + [
            {'robot_description_semantic': robot_description_semantic_5dof},
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", manipulator, "' == '5DOF'"])
        ),
    )

    # ---- move_group for 6DOF ----
    move_group_6dof = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=move_group_common_params + [
            {'robot_description_semantic': robot_description_semantic_6dof},
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", manipulator, "' == '6DOF'"])
        ),
    )

    # ---- RViz (one per SRDF variant so robot_description_semantic is correct) ----
    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    rviz_5dof = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'robot_description': robot_description,
                     'robot_description_semantic': robot_description_semantic_5dof,
                     'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", manipulator, "' == '5DOF'"])),
    )

    rviz_6dof = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'robot_description': robot_description,
                     'robot_description_semantic': robot_description_semantic_6dof,
                     'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", manipulator, "' == '6DOF'"])),
    )

    return LaunchDescription([
        manipulator_arg,
        use_real_hw_arg,
        use_sim_time_arg,
        rsp_node,
        ros2_control_node,
        spawn_jsb,
        spawn_base,
        spawn_manip,
        move_group_5dof,
        move_group_6dof,
        rviz_5dof,
        rviz_6dof,
    ])
