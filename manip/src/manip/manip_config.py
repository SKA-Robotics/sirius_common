import rospy
import numpy as np
from typing import List

class ManipConfig:
    # Control params
    control_frequency: float = 30
    command_timeout: float = 0.3
    servo_gain: np.ndarray = np.array([20, 20, 20, 10, 10, 10])
    singularity_avoidance_A: float = 1.6    # Values based on paper
    singularity_avoidance_B: float = 1.491  # Values based on paper
    max_setpoint_position_distance: float = 0.05
    max_setpoint_orientation_distance: float = 0.12
    trajectory_duration: float = 6
    max_ev: np.ndarray = np.array([0.2, 0.2, 0.2, 0.4, 0.4, 0.4]) # Maximum cartesian velocity of the end effector
    max_qd: np.ndarray = np.array([0.3, 0.3, 0.3, 0.6, 0.6, 1.0]) # Maximum velocity of each of the arm joints

    # Robot params
    robot_urdf_path: str = "package://manip/urdf/sirius2_manip.urdf.xacro"
    robot_joint_names: List[str] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    base_frame_id: str = "base_link"
    ee_frame_id: str = "tool"
    twist_cmd_frame_id: str = "tool"

    # ROS params
    joy_topic: str = "/joy"
    twist_topic: str = "/twist_cmd"
    pose_topic: str = "/pose_cmd"
    joint_topic: str = "/joint_cmd"
    preset_request_topic: str = "/preset_request"
    gripper_cmd_topic: str = "/gripper_cmd"

    robot_state_topic: str = "/manip/joint_states"
    robot_command_topic: str = "/manip/set_joint_states"


DEFAULT_CONFIG = ManipConfig()


def load_ros_params() -> ManipConfig:
    """
    Loads manipulator configuration parameters from the ROS parameter server.
    If a parameter is not found on the server, the default value from ManipConfig
    will be used.
    """
    config = ManipConfig() # Start with default values

    # Control parameters
    config.control_frequency = rospy.get_param("control_frequency", config.control_frequency)
    config.command_timeout = rospy.get_param("command_timeout", config.command_timeout)
    config.servo_gain = np.array(rospy.get_param("servo_gain", config.servo_gain.tolist()))
    config.singularity_avoidance_A = rospy.get_param("singularity_avoidance_A", config.singularity_avoidance_A)
    config.singularity_avoidance_B = rospy.get_param("singularity_avoidance_B", config.singularity_avoidance_B)
    config.max_setpoint_position_distance = rospy.get_param("max_setpoint_position_distance", config.max_setpoint_position_distance)
    config.max_setpoint_orientation_distance = rospy.get_param("max_setpoint_orientation_distance", config.max_setpoint_orientation_distance)
    config.trajectory_duration = rospy.get_param("trajectory_duration", config.trajectory_duration)
    config.max_ev = np.array(rospy.get_param("max_ev", config.max_ev.tolist()))
    config.max_qd = np.array(rospy.get_param("max_qd", config.max_qd.tolist()))

    # Robot parameters
    config.robot_urdf_path = rospy.get_param("robot_urdf_path", config.robot_urdf_path)
    config.robot_joint_names = rospy.get_param("robot_joint_names", config.robot_joint_names)
    config.base_frame_id = rospy.get_param("base_frame_id", config.base_frame_id)
    config.ee_frame_id = rospy.get_param("ee_frame_id", config.ee_frame_id)
    config.twist_cmd_frame_id = rospy.get_param("twist_cmd_frame_id", config.twist_cmd_frame_id)

    # ROS topic names
    config.joy_topic = rospy.get_param("joy_topic", config.joy_topic)
    config.twist_topic = rospy.get_param("twist_topic", config.twist_topic)
    config.pose_topic = rospy.get_param("pose_topic", config.pose_topic)
    config.joint_topic = rospy.get_param("joint_topic", config.joint_topic)
    config.gripper_cmd_topic = rospy.get_param("gripper_cmd_topic", config.gripper_cmd_topic)
    config.preset_request_topic = rospy.get_param("preset_request_topic", config.preset_request_topic)
    config.robot_state_topic = rospy.get_param("robot_state_topic", config.robot_state_topic)
    config.robot_command_topic = rospy.get_param("robot_command_topic", config.robot_command_topic)

    return config