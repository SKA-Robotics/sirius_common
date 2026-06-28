import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import Trajectory
from typing import Optional
import qpsolvers as qp
from manip.manip_config import ManipConfig
import scipy.sparse as sp


class TwistController:

    def __init__(self, robot: rtb.ERobot, config: ManipConfig):
        self.robot = robot
        self.config = config

    def compute_twist_control(self, ev: np.ndarray, pos_e: float,
                              rot_e: float) -> np.ndarray:
        """
        MMC - Manipulability Motion Control
        https://jhavl.github.io/mmc/

        Computes joint velocities to achieve a desired end-effector twist.

        Args:
            robot: roboticstoolbox robot model
            ev: desired end-effector twist
            pos_e: position error
            rot_e: rotation error

        Returns:
            qd: joint velocities
        """

        Y = 0.001
        n = self.robot.n
        Q = np.eye(n + 6)
        Q[:n, :n] *= Y
        Q[n:, n:] = np.diag(
            np.array([10, 10, 10, 1, 1, 1]) * (1 / (pos_e + rot_e) + 0.01))
        Aeq = np.c_[self.robot.jacobe(self.robot.q), np.eye(6)]
        beq = ev.reshape((6, ))
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)
        ps = 0.05
        pi = 0.9
        Ain[:n, :n], bin[:n] = self.robot.joint_velocity_damper(ps, pi, n)
        c = np.r_[-self.robot.jacobm().reshape((n, )), np.zeros(6)]

        qdlim = self.config.max_qd

        lb = -np.r_[qdlim[:n], 10 * np.ones(6)]
        ub = np.r_[qdlim[:n], 10 * np.ones(6)]

        Q = sp.csc_matrix(Q)
        Ain = sp.csc_matrix(Ain)
        Aeq = sp.csc_matrix(Aeq)

        qd = qp.solve_qp(Q,
                         c,
                         Ain,
                         bin,
                         Aeq,
                         beq,
                         lb=lb,
                         ub=ub,
                         solver="clarabel")
        return qd[:n]


class PoseServo:

    def __init__(self, robot: rtb.ERobot, gain: np.ndarray):
        self.robot = robot
        self.gain = gain

    def compute_pose_control(self, Td: np.ndarray) -> np.ndarray:
        """
        Computes end-effector velocities that perform servoing of the end-effector to a desired pose.

        Args:
            robot: roboticstoolbox robot model
            Td: desired end-effector pose, relative to the robot base frame
            gain: gain vector for the pose controller

        Returns:
            ev: end-effector velocities
        """
        Te = self.robot.fkine(self.robot.q, tool=self.robot.tool).A

        ev, _ = rtb.p_servo(Te, Td, self.gain, threshold=0.001, method="rpy")

        return ev


class TrajectoryExecutor:

    def __init__(self):
        self._trajectory: Optional[Trajectory] = None
        self._idx = 0
        self.running = False

    def set_trajectory(self, trajectory: Trajectory):
        self._idx = 0
        self._trajectory = trajectory
        self.running = True

    def step(self) -> np.ndarray:
        if not self.running:
            raise IndexError(
                "Called step() on TrajectoryExecutor which is not running")
        q = self._trajectory.q[self._idx]
        self._idx += 1
        if self._idx == len(self._trajectory):
            self.running = False
        return q
