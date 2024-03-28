from ik import SiriusII_IKSolver, ManipPose
from manip_interface import ManipInterface
from motion_interpolation import InterpolationSettings
from motion_strategies import CartesianMotion, JointspaceMotion, IncrementalMotion


class SiriusManip:

    def __init__(self, manip_interface: ManipInterface):
        self.manip_interface = manip_interface
        self.params = self.manip_interface.get_manip_params()
        self.solver = self._create_ik_solver(self.params)

    def _create_ik_solver(self, params):
        return SiriusII_IKSolver(params.joint_names(), params.link_lengths(), params.joint_limits())

    def move_cartesian(self, target_pose: ManipPose):
        self._move(target_pose, CartesianMotion, "cartesian")

    def move_jointspace(self, target_pose: ManipPose):
        self._move(target_pose, JointspaceMotion, "jointspace")

    def _get_controlmode_settings(self, mode_name: str):
        mode_params = self.params.control_mode_params(mode_name)
        interpolation_settings = InterpolationSettings.from_params(mode_params)
        rate = mode_params["interpolation_rate"]
        return interpolation_settings, rate

    def _move(self, target_pose: ManipPose, motion_strategy_class, mode_name: str):
        interpolation_settings, rate = self._get_controlmode_settings(mode_name)
        motion = motion_strategy_class(target_pose, interpolation_settings, self.solver, rate)
        motion.execute(self.manip_interface)

    def get_ik_solver(self):
        return self.solver

    def move_incremental(self, pose_delta: ManipPose):
        motion = IncrementalMotion(pose_delta, self.solver)
        motion.execute(self.manip_interface)