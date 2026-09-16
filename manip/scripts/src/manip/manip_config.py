import numpy as np
from typing import List


class ManipConfig:
    # Control params
    control_frequency: float = 30
    command_timeout: float = 0.3
    servo_gain: np.ndarray = np.array([20, 20, 20, 10, 10, 10])
    singularity_avoidance_A: float = 1.6
    singularity_avoidance_B: float = 1.491
    max_setpoint_position_distance: float = 0.05
    max_setpoint_orientation_distance: float = 0.12
    trajectory_duration: float = 6
    max_ev: np.ndarray = np.array([0.2, 0.2, 0.2, 0.4, 0.4, 0.4])
    max_qd: np.ndarray = np.array([0.3, 0.3, 0.3, 0.6, 0.6, 1.0])

    # Robot params
    robot_urdf_path: str = 'package://manip/urdf/sirius2_manip.urdf.xacro'
    robot_joint_names: List[str] = [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    base_frame_id: str = 'base_link'
    ee_frame_id: str = 'tool'
    twist_cmd_frame_id: str = 'tool'

    # ROS topic names
    joy_topic: str = '/joy'
    twist_topic: str = '/twist_cmd'
    pose_topic: str = '/pose_cmd'
    joint_topic: str = '/joint_cmd'
    preset_request_topic: str = '/preset_request'
    gripper_cmd_topic: str = '/gripper_cmd'
    robot_state_topic: str = '/manip/joint_states'
    robot_command_topic: str = '/manip/set_joint_states'


DEFAULT_CONFIG = ManipConfig()


def load_ros_params(node=None) -> ManipConfig:
    """Load ManipConfig from a rclpy Node's parameter server (ROS2).

    Pass the rclpy Node as ``node``. If *node* is None the default config is
    returned (useful for testing without a live node).
    """
    config = ManipConfig()
    if node is None:
        return config

    def _get(name, default):
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        return node.get_parameter(name).value

    config.control_frequency = _get('control_frequency', config.control_frequency)
    config.command_timeout = _get('command_timeout', config.command_timeout)
    config.servo_gain = np.array(
        _get('servo_gain', config.servo_gain.tolist()))
    config.singularity_avoidance_A = _get(
        'singularity_avoidance_A', config.singularity_avoidance_A)
    config.singularity_avoidance_B = _get(
        'singularity_avoidance_B', config.singularity_avoidance_B)
    config.max_setpoint_position_distance = _get(
        'max_setpoint_position_distance', config.max_setpoint_position_distance)
    config.max_setpoint_orientation_distance = _get(
        'max_setpoint_orientation_distance', config.max_setpoint_orientation_distance)
    config.trajectory_duration = _get(
        'trajectory_duration', config.trajectory_duration)
    config.max_ev = np.array(_get('max_ev', config.max_ev.tolist()))
    config.max_qd = np.array(_get('max_qd', config.max_qd.tolist()))

    config.robot_urdf_path = _get('robot_urdf_path', config.robot_urdf_path)
    config.robot_joint_names = _get(
        'robot_joint_names', config.robot_joint_names)
    config.base_frame_id = _get('base_frame_id', config.base_frame_id)
    config.ee_frame_id = _get('ee_frame_id', config.ee_frame_id)
    config.twist_cmd_frame_id = _get(
        'twist_cmd_frame_id', config.twist_cmd_frame_id)

    config.joy_topic = _get('joy_topic', config.joy_topic)
    config.twist_topic = _get('twist_topic', config.twist_topic)
    config.pose_topic = _get('pose_topic', config.pose_topic)
    config.joint_topic = _get('joint_topic', config.joint_topic)
    config.gripper_cmd_topic = _get(
        'gripper_cmd_topic', config.gripper_cmd_topic)
    config.preset_request_topic = _get(
        'preset_request_topic', config.preset_request_topic)
    config.robot_state_topic = _get(
        'robot_state_topic', config.robot_state_topic)
    config.robot_command_topic = _get(
        'robot_command_topic', config.robot_command_topic)

    return config
