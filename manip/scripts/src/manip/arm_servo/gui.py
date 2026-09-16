import threading
import time
import curses
import numpy as np

from manip.arm_servo.arm_controller import DEFAULT_CONFIG, ArmController
from manip.arm_servo.ros_command_receiver import RosCommandReceiver
from manip.arm_servo.ros_robot_interface import RosRobotInterface
from manip.arm_servo.ros_visualizer import RosVisualizer
from manip.arm_servo.command import CommandType

class ArmControllerGui:
    def __init__(self, arm_controller: ArmController, window: curses.window):
        self.arm_controller = arm_controller
        self.window = window
        self._draw_canvas()
    
    def _draw_canvas(self):
        self.window.addstr(0, 0, f"Arm Controller v1.0", curses.color_pair(1) | curses.A_BOLD)
        self.window.refresh()


    def update(self):
        state = np.array(self.arm_controller.robot_interface.get_state()[0])
        command = self.arm_controller.robot.q
        error = np.linalg.norm(state - command)
        ee_pose = self.arm_controller.ee_axes.T
        goal_pose = self.arm_controller.goal.T
        ee_pose_error = 100 * np.linalg.norm(ee_pose[0:3, 3] - goal_pose[0:3, 3])

        self.window.addstr(1, 0, f"Robot state: {list(np.round(self.arm_controller.robot_interface.get_state()[0], 2))}", curses.color_pair(1))
        self.window.addstr(2, 0, f"Robot command: {np.round(self.arm_controller.robot.q, 2)}", curses.color_pair(1))
        self.window.addstr(3, 0, f"Cmd vs state MSE: {np.round(error, 2)}", curses.color_pair(1))
        self.window.addstr(5, 0, f"EE position error [cm]: {np.round(ee_pose_error, 2)}", curses.color_pair(1))

        manipulability = 100 * self.arm_controller.robot.manipulability(self.arm_controller.robot.q)
        self.window.addstr(6, 0, f"Manipulability: {np.round(manipulability, 2):<10}", curses.color_pair(1))

        mode_str = "other mode"
        if self.arm_controller.command is not None:
            if self.arm_controller.command.type == CommandType.END_EFFECTOR_POSE_CMD:
                mode_str = "pose servo (IK)"
            elif self.arm_controller.command.type == CommandType.END_EFFECTOR_TWIST_CMD:
                mode_str = "twist servo (IK)"
            elif self.arm_controller.command.type == CommandType.JOINT_VELOCITY_CMD:
                mode_str = "joint control (FK)"
            elif self.arm_controller.command.type == CommandType.TRAJECTORY_CMD:
                mode_str = "trajectory"
            self.window.addstr(8, 0, f"Mode: {mode_str:<20}", curses.color_pair(1))
        
        self.window.addstr(10, 0, f"Compute time [ms]: {round(self.arm_controller.compute_duration * 1000):<5}rate: {round(1 / self.arm_controller.loop_duration, 2):<5}", curses.color_pair(1))
        self.window.refresh()


def main(window: curses.window):
    curses.start_color()
    curses.use_default_colors()
    curses.curs_set(0)
    curses.init_pair(1, curses.COLOR_WHITE, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_GREEN, -1)
    curses.init_pair(4, curses.COLOR_BLUE, -1)

    config = DEFAULT_CONFIG

    # Start ros2 integration
    if config.ros_version == "ros2":
        from ros_middleware.ros2 import Ros2Middleware
        ros = Ros2Middleware("manip_control")
    elif config.ros_version == "ros1":
        from ros_middleware.ros1 import Ros1Middleware
        ros = Ros1Middleware("manip_control")
    else:
        raise ValueError(f"Invalid ROS version: {config.ros_version}. Must be either 'ros1' or 'ros2'.")

    ros.start()

    robot_interface = RosRobotInterface(ros, config.robot_joint_names, config.robot_state_topic, config.robot_command_topic)
    command_interface = RosCommandReceiver(ros, config.twist_topic, config.pose_topic, config.joint_topic, config.preset_request_topic)

    if config.use_sim:
        from swift import Swift
        env = Swift()
    else:
        env = None

    # viz = None
    viz = RosVisualizer(ros, "/manip_goal")

    arm_controller = ArmController(config, robot_interface, command_interface, env, viz)

    if config.use_sim:
        env.launch(realtime=False, browser=config.sim_browser)
        env.add(arm_controller.robot)
        env.add(arm_controller.ee_axes)
        env.add(arm_controller.goal)

    def run_arm_controller():
        try:
            arm_controller.loop()

        except KeyboardInterrupt:
            pass
        finally:
            if config.use_sim:
                env.close()

            ros.stop()


    main_thread = threading.Thread(target=run_arm_controller)
    main_thread.start()

    gui = ArmControllerGui(arm_controller, window)

    try:
        while main_thread.is_alive():
            gui.update()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        main_thread.join()
        ros.stop()
        


if __name__ == "__main__":
    curses.wrapper(main)