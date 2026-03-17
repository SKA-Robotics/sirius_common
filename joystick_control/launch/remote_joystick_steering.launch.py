from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joystick_type_arg = DeclareLaunchArgument(
        'joystick_type',
        default_value='STANDARD',
        description='Type of joystick controller defined in joysticks.yaml',
    )

    joysticks_config = PathJoinSubstitution([
        FindPackageShare('joystick_control'), 'config', 'joysticks.yaml'
    ])
    steering_modes_config = PathJoinSubstitution([
        FindPackageShare('joystick_control'), 'config', 'steering_modes.yaml'
    ])
    diff_drive_config = PathJoinSubstitution([
        FindPackageShare('joystick_control'), 'config', 'diff_drive.yaml'
    ])

    joy_multiplexer_node = Node(
        package='joystick_control',
        executable='joy_multiplexer',
        name='joy_multiplexer',
        output='screen',
        parameters=[
            joysticks_config,
            steering_modes_config,
            {'joystick_type': LaunchConfiguration('joystick_type')},
        ],
    )

    joy_diff_drive_node = Node(
        package='joystick_control',
        executable='joy_diff_drive',
        name='joy_diff_drive',
        output='screen',
        parameters=[
            joysticks_config,
            diff_drive_config,
            {'joystick_type': LaunchConfiguration('joystick_type')},
        ],
        remappings=[
            ('max_angular_rate',
             '/sirius/controller/wheels/angular/z/max_velocity'),
            ('max_linear_rate',
             '/sirius/controller/wheels/linear/x/max_velocity'),
        ],
    )

    # joy_5dof_manipulator_node = Node(
    #     package='joystick_control',
    #     executable='joy_5dof_manipulator',
    #     name='joy_5dof_manipulator',
    #     output='screen',
    #     parameters=[
    #         joysticks_config,
    #         PathJoinSubstitution([
    #             FindPackageShare('joystick_control'),
    #             'config', '5dof_manipulator.yaml'
    #         ]),
    #         {'joystick_type': LaunchConfiguration('joystick_type')},
    #     ],
    # )

    return LaunchDescription([
        joystick_type_arg,
        joy_multiplexer_node,
        joy_diff_drive_node,
        # joy_5dof_manipulator_node,
    ])