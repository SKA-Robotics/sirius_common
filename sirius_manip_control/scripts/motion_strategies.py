from abc import ABC, abstractmethod
from math import pi

import numpy as np
from geometry_msgs.msg import PoseStamped

from manip_interface import ManipInterface
from motion_interpolation import InterpolationSettings, MotionInterpolator
from ik import IKSolver, ManipPose, ManipJointState


class MotionStrategy(ABC):

    @abstractmethod
    def execute(self, manip_interface: ManipInterface):
        pass


class InterpolatedMotion(MotionStrategy):

    def __init__(self, target_pose: ManipPose,
                 interpolation_settings: InterpolationSettings,
                 ik_solver: IKSolver, rate: float):
        self.motion_interpolator = MotionInterpolator(interpolation_settings)
        self.ik_solver = ik_solver
        self.target_pose = target_pose
        self.loop_delay = 1.0 / rate

    def execute(self, manip_interface: ManipInterface):
        self._initialize_execution(manip_interface)
        while self.motion_interpolator.is_not_done():
            self._step(manip_interface)

    def _initialize_execution(self, manip_interface: ManipInterface):
        position = self._calculate_start_coords(manip_interface)
        end_position = self._calculate_end_coords(manip_interface)
        self.motion_interpolator.set_movement(position, end_position)

    def _step(self, manip_interface: ManipInterface):
        position = self.motion_interpolator.movement_step(self.loop_delay)
        self._move_to_position(position, manip_interface)
        manip_interface.sleep(self.loop_delay)

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        pass

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        pass

    def _move_to_position(self, position, manip_interface: ManipInterface):
        pass


class CartesianMotion(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        pose = self.ik_solver.get_FK_solution(manip_interface.get_jointstate())
        return pose.to_list()

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        return self.target_pose.to_list()

    def _move_to_position(self, position, manip_interface: ManipInterface):
        pose = ManipPose.from_list(position)
        jointstate = self.ik_solver.get_IK_solution(pose)
        manip_interface.set_jointstate(jointstate)


class JointspaceMotion(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        return manip_interface.get_jointstate().position

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        return self.ik_solver.get_IK_solution(self.target_pose).position

    def _move_to_position(self, position, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        jointstate.position = position
        manip_interface.set_jointstate(jointstate)


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = (np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) -
          np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2))
    qy = (np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) +
          np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2))
    qz = (np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) -
          np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2))
    qw = (np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) +
          np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2))
    return [qx, qy, qz, qw]


class IncrementalMotion(MotionStrategy):

    def __init__(self, start: ManipPose, delta: ManipPose,
                 ik_solver: IKSolver):
        self.startPose = start
        self.deltaPose = delta
        self.solver = ik_solver
        self.targetPose = ManipPose.from_list(
            [x1 + x2 for x1, x2 in zip(start.to_list(), delta.to_list())])
        self.targetJointstate = self.solver.get_IK_solution(self.targetPose)
        self.endPose = self.targetPose

    def execute(self, manip_interface: ManipInterface):
        manip_interface.set_jointstate(self.targetJointstate)

    def get_end_pose(self):
        return self.endPose
