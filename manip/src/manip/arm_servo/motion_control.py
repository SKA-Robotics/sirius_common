import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import Trajectory
from typing import Optional



class TwistController:
    def __init__(self, robot: rtb.ERobot, A: float, B: float):
        self.robot = robot
        self.A = A
        self.B = B

    def compute_twist_control(self, ev: np.ndarray) -> np.ndarray:
        """
        Computes joint velocities to achieve a desired end-effector twist.

        Args:
            robot: roboticstoolbox robot model
            ev: desired end-effector twist

        Returns:
            qd: joint velocities
        """
        J_pinv = self._damped_jacobian(self.robot.q)
        qd = J_pinv @ ev

        return qd
    
    def _damped_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Computes the damped Jacobian for a robot. The dampening allows for better numerical conditioning around arm singularities. Based on https://www.researchgate.net/publication/356023999_Singularity_Avoidance_Strategies_for_Target_Tracking_Visual_Servo_Systems

        Args:
            q: joint configuration for which to compute the Jacobian

        Returns:
            J_pinv: damped Jacobian
        """

        J = self.robot.jacobe(q, tool=self.robot.tool)
        detJJT = np.linalg.det(J @ J.T)
        detJJT = max(0.0, detJJT) # Due to numerical errors detJJT sometimes comes out negative - clamp it
        c = np.sqrt(detJJT)
        l = self.A / (np.tan(c + self.B))
        return J.T @ np.linalg.inv(J @ J.T + l * np.eye(J.shape[0]))


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
            raise IndexError("Called step() on TrajectoryExecutor which is not running")
        q = self._trajectory.q[self._idx]
        self._idx += 1
        if self._idx == len(self._trajectory):
            self.running = False
        return q
