from enum import Enum

class CommandType(Enum):
    JOINT_VELOCITY_CMD = "joint_velocity_cmd"
    END_EFFECTOR_TWIST_CMD = "end_effector_twist_cmd"
    END_EFFECTOR_POSE_CMD = "end_effector_pose_cmd"
    TRAJECTORY_CMD = "trajectory_cmd"

class Command:
    def __init__(self, type: CommandType, data: dict, timestamp: float):
        self.timestamp = timestamp
        self.type = type
        self.data = data