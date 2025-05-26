import time
import rospy
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import roboticstoolbox as rtb
from typing import List
from manip.arm_servo.command import Command, CommandType
from manip.arm_servo.ros_robot_interface import RosRobotInterface
from manip.arm_servo.ros_command_receiver import RosCommandReceiver
from manip.arm_servo.motion_control import TwistController, PoseServo, TrajectoryExecutor
from manip.arm_servo.ros_visualizer import RosVisualizer
from manip.arm_servo.utils import load_urdf, angle_axis_error, clamp_velocity
from manip.manip_config import ManipConfig, DEFAULT_CONFIG



class ArmController:
    def __init__(self, config: ManipConfig,  robot_model: rtb.ERobot, robot_interface: RosRobotInterface, command_interface: RosCommandReceiver, viz):
        self.config = config
        self.command = None
        self.robot_interface = robot_interface
        self.command_interface = command_interface
        self.robot = robot_model
        self.pose_servo = PoseServo(self.robot, gain=config.servo_gain)
        self.twist_controller = TwistController(self.robot, config.singularity_avoidance_A, config.singularity_avoidance_B)
        self.trajectory_executor = TrajectoryExecutor()
        self.goal = sg.Axes(0.1)
        self.ee_axes = sg.Axes(0.1)
        self.command_timeout = config.command_timeout
        self.viz = viz
        self.compute_duration = 0
        self.loop_duration = 100

    def loop(self):
        self.robot.q, _ = self.robot_interface.get_state()
        self.goal.T = self.robot.fkine(self.robot.q, tool=self.robot.tool)

        dt = 1 / self.config.control_frequency
        start_time = time.monotonic()
        while not rospy.is_shutdown():
            loop_start_time = time.monotonic()
            self.command = self.command_interface.pop_command()
            if self.trajectory_executor.running:
                self.robot.q = self.trajectory_executor.step()
                self.goal.T = self.robot.fkine(self.robot.q, tool=self.robot.tool).A
            else:
                qd = self.compute_control(dt)
                self.robot.q = self.robot.q + qd * dt

            # Clamp joint values to the joint limits
            for i in range(self.robot.n):
                self.robot.q[i] = max(min(self.robot.q[i], self.robot.qlim[1][i]), self.robot.qlim[0][i])

            self.robot_interface.set_joint_state(q=self.robot.q)
            
            self.ee_axes.T = self.robot.fkine(self.robot.q, tool=self.robot.tool)
            compute_end_time = time.monotonic()

            if self.viz:
                self.viz.visualize_goal_pose(self.goal.T, "base_link")
            
            # Try to make the loop run at the correct frequency
            sleep_time = dt - ((time.monotonic() - start_time) % dt)
            if sleep_time > 0:
                time.sleep(sleep_time)

            loop_end_time = time.monotonic()
            self.compute_duration = compute_end_time - loop_start_time
            self.loop_duration = loop_end_time - loop_start_time

    
    def compute_control(self, dt: float) -> np.ndarray:
        """
        Calculates the joint velocities required to execute the command.

        Returns:
            np.ndarray: The joint velocities required to execute the command.
        """

        if self.command is not None and self.command.timestamp < time.time() - self.command_timeout:
            print(f"Command timed out: {self.command.timestamp} < {time.time() - self.command_timeout}")
            self.command = None

        if self.command is None:
            return np.zeros(self.robot.n)

        if self.command.type == CommandType.END_EFFECTOR_POSE_CMD:
            self.goal.T = self.command.data
            ev = self.pose_servo.compute_pose_control(self.goal.T)
            qd = self.twist_controller.compute_twist_control(ev)

        elif self.command.type == CommandType.END_EFFECTOR_TWIST_CMD:
            new_goal_T = self.goal.T
            new_goal_T = new_goal_T @ sm.SE3.Trans(self.command.data[0] * dt, self.command.data[1] * dt, self.command.data[2] * dt).A
            new_goal_T = new_goal_T @ sm.SE3.RPY(self.command.data[3] * dt, self.command.data[4] * dt, self.command.data[5] * dt).A
            # new_goal_T_position_error = np.linalg.norm(self.ee_axes.T[0:3, 3] - new_goal_T[0:3, 3])
            # if new_goal_T_position_error <= self.config.max_setpoint_distance:
            #     self.goal.T = new_goal_T # The maximum distance from ee to goal is limited to avoid going far out of range of the arm
            new_goal_error = angle_axis_error(self.ee_axes.T, new_goal_T)
            position_error = np.linalg.norm(new_goal_error[0:3])
            orientation_error = np.linalg.norm(new_goal_error[3:6])
            if position_error <= self.config.max_setpoint_position_distance and orientation_error <= self.config.max_setpoint_orientation_distance:
                self.goal.T = new_goal_T # The maximum distance from ee to goal is limited to avoid going far out of range of the arm
            ev = self.pose_servo.compute_pose_control(self.goal.T)
            ev = clamp_velocity(ev, self.config.max_ev)
            qd = self.twist_controller.compute_twist_control(ev)

        elif self.command.type == CommandType.JOINT_VELOCITY_CMD:
            if len(self.command.data) != self.robot.n:
                print(f"Command received for {len(self.command.data)} joints, but robot has {self.robot.n} joints")
                self.command = Command(CommandType.JOINT_VELOCITY_CMD, np.zeros(self.robot.n), time.time())
            self.goal.T = self.robot.fkine(self.robot.q, tool=self.robot.tool).A # reset the end effector pose goal
            # self.robot.q, _ = self.robot_interface.get_state() # Set current robot state to actual state read from hardware
            qd = self.command.data

        elif self.command.type == CommandType.TRAJECTORY_CMD:
            trajectory = rtb.jtraj(self.robot.q, np.array(self.command.data), int(self.config.trajectory_duration/dt))
            self.trajectory_executor.set_trajectory(trajectory)
            return np.zeros(self.robot.n)

        else:
            print(f"Unknown command type: {self.command.type}")
            return np.zeros(self.robot.n)
        
        qd = clamp_velocity(qd, self.config.max_qd)
        return qd


    

def main(config: ManipConfig=DEFAULT_CONFIG):

    rospy.init_node("manip_control")

    robot_model = load_urdf(config.robot_urdf_path)
    robot_interface = RosRobotInterface(config.robot_joint_names, config.robot_state_topic, config.robot_command_topic)
    command_interface = RosCommandReceiver(config, robot_model)

    # viz = None
    viz = RosVisualizer("/manip_goal")

    arm_controller = ArmController(config, robot_model, robot_interface, command_interface, viz)

    try:
        arm_controller.loop()

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()