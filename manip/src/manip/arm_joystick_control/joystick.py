from enum import Enum
from typing import List, Dict
from manip.arm_joystick_control.ros_joy_receiver import RosJoyReceiver
from manip.arm_joystick_control.ros_command_sender import RosCommandSender
from manip.arm_joystick_control.gripper_controller import GripperController
from manip.arm_joystick_control.utils import max_abs, trig_to_axis, JoystickTranslator
from manip.manip_config import ManipConfig, DEFAULT_CONFIG
from sensor_msgs.msg import JointState
import time
import rospy

MANIP_PRESET_DATABASE = {
    "left": [0.0, -0.5, 1.85, -1.53, 0.96, -0.06],
    "up": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "down": [0.0, 1.0, 1.462, -1.618, 0.6327, 0.0],
    "right": [-2.3279, 0.3627, 1.4217, -1.2624, 1.0883, 0.0651]
}

class JoystickControl():

    class Axis(Enum):
        LINEAR_X = 0
        LINEAR_Y = 1
        LINEAR_Z = 2
        ANGULAR_X = 3
        ANGULAR_Y = 4
        ANGULAR_Z = 5
        GRIPPER = 6
        JOINT_1 = 7
        JOINT_2 = 8
        JOINT_3 = 9
        JOINT_4 = 10
        JOINT_5 = 11
        JOINT_6 = 12

    class Button(Enum):
        SET_FRAME_TOOL = 1
        SET_FRAME_BASE = 2
        SET_JOINT_MODE = 3
        CHANGE_MOVEMENT_MODE = 4
        LEFT = 5
        UP = 6
        DOWN = 7
        RIGHT = 8
        SET_PRESET = 9

    class SpaceMode(Enum):
        CARTESIAN = 0
        JOINT = 1

    class MovementMode(Enum):
        LINEAR = 0
        ANGULAR = 1

    def __init__(self, config: ManipConfig = DEFAULT_CONFIG):
        self.config = config
        self.movement_mode = self.MovementMode.LINEAR
        self.space_mode = self.SpaceMode.JOINT
        self.frame = config.base_frame_id
        self.gui = None

        rospy.init_node("joystick_control")
        self.gripper_controller = GripperController()
        self.command_sender = RosCommandSender(config.twist_topic, config.joint_topic, config.gripper_cmd_topic, config.preset_request_topic)
        self.joy_receiver = RosJoyReceiver(config.joy_topic)

    def run(self):
        self.joy_receiver.register_callback(self.receive_command)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def receive_command(self, raw_axes: List[float], raw_buttons: List[int]):
        input = JoystickTranslator().translate({
            "axes": raw_axes,
            "buttons": raw_buttons
        })
        buttons = self._process_buttons(input)
        self._handle_buttons(buttons)
        
        rospy.loginfo(buttons)
        
        if buttons[self.Button.SET_PRESET]:
            # SETTING UP PRESET

            current_manip_state = {}
            while not (len(current_manip_state) == 6):
                msg = rospy.wait_for_message(self.config.robot_state_topic, JointState)
                # joint_names = self.confing.robot_joint_names
                names = msg.name
                for i in range(len(names)):
                    current_manip_state[names[i]] = msg.position[i]

            joint_names = self.config.robot_joint_names

            position = []
            for name in joint_names:
                position.append(current_manip_state[name])


            key = None
            if buttons[self.Button.LEFT]:
                key = "left"
            elif buttons[self.Button.UP]:
                key = "up"
            elif buttons[self.Button.DOWN]:
                key = "down"
            elif buttons[self.Button.RIGHT]:
                key = "right"

            MANIP_PRESET_DATABASE[key] = position
        elif buttons[self.Button.LEFT]:
            self._send_preset_request("left")
        elif buttons[self.Button.UP]:
            self._send_preset_request("up")
        elif buttons[self.Button.DOWN]:
            self._send_preset_request("down")
        elif buttons[self.Button.RIGHT]:
            self._send_preset_request("right")
        else:
            self._update_gui(raw_axes, raw_buttons)
            axes  = self._process_axes(input)
            self._publish_command(axes)
            self._control_gripper(axes[self.Axis.GRIPPER])

    def _process_axes(self, input: Dict[str, float]) -> Dict[Axis, float]:
        return {
            self.Axis.LINEAR_X: -input["left_stick_vertical"],
            self.Axis.LINEAR_Y: max_abs(-input["left_stick_horizontal"], -input["right_stick_horizontal"]),
            self.Axis.LINEAR_Z: -input["right_stick_vertical"],
            self.Axis.ANGULAR_X: input["left_stick_horizontal"],
            self.Axis.ANGULAR_Y: max_abs(input["left_stick_vertical"], input["right_stick_vertical"]),
            self.Axis.ANGULAR_Z: -input["right_stick_horizontal"],
            self.Axis.GRIPPER: max_abs(input["left_trigger"], input["right_trigger"]),
            self.Axis.JOINT_1: -input["left_stick_horizontal"],
            self.Axis.JOINT_2: -input["left_stick_vertical"],
            self.Axis.JOINT_3: -input["right_stick_vertical"],
            self.Axis.JOINT_4: input["right_stick_horizontal"],
            self.Axis.JOINT_5: -input["left_stick_vertical"],
            self.Axis.JOINT_6: input["left_stick_horizontal"],
        }

    def _process_buttons(self, input: Dict[str, float]) -> Dict[Button, bool]:
        return {
            self.Button.SET_FRAME_BASE:
                input["start_button"] and (input["left_bumper"] or input["right_bumper"]),
            self.Button.SET_FRAME_TOOL:
                input["start_button"] and not (input["left_bumper"] or input["right_bumper"]),
            self.Button.SET_JOINT_MODE: input["back_button"],
            self.Button.CHANGE_MOVEMENT_MODE: input["left_bumper"] or input["right_bumper"],
            self.Button.LEFT: input["left_cross"],
            self.Button.UP: input["up_cross"],
            self.Button.DOWN: input["down_cross"],
            self.Button.RIGHT: input["right_cross"],
            self.Button.SET_PRESET: input["start_button"] and input["back_button"]
        }

    def _handle_buttons(self, buttons: Dict[Button, bool]):
        if buttons[self.Button.SET_FRAME_TOOL]:
            self.frame = self.config.twist_cmd_frame_id
            self.space_mode = self.SpaceMode.CARTESIAN
        elif buttons[self.Button.SET_FRAME_BASE]:
            self.frame = self.config.base_frame_id
            self.space_mode = self.SpaceMode.CARTESIAN
        elif buttons[self.Button.SET_JOINT_MODE]:
            self.space_mode = self.SpaceMode.JOINT

        if buttons[self.Button.CHANGE_MOVEMENT_MODE]:
            self.movement_mode = self.MovementMode.ANGULAR
        else:
            self.movement_mode = self.MovementMode.LINEAR

    def _publish_command(self, axes: Dict[Axis, float]):
        if self.space_mode == self.SpaceMode.CARTESIAN:
            if self.movement_mode == self.MovementMode.LINEAR:
                command = [
                    axes[self.Axis.LINEAR_X] * self.config.max_ev[0],
                    axes[self.Axis.LINEAR_Y] * self.config.max_ev[1],
                    axes[self.Axis.LINEAR_Z] * self.config.max_ev[2],
                    0.0,
                    0.0,
                    0.0,
                ]
            elif self.movement_mode == self.MovementMode.ANGULAR:
                command = [
                    0.0,
                    0.0,
                    0.0,
                    axes[self.Axis.ANGULAR_X] * self.config.max_ev[3],
                    axes[self.Axis.ANGULAR_Y] * self.config.max_ev[4],
                    axes[self.Axis.ANGULAR_Z] * self.config.max_ev[5],
                ]
            self.command_sender.send_twist_command(command, self.frame)

        elif self.space_mode == self.SpaceMode.JOINT:
            if self.movement_mode == self.MovementMode.LINEAR:
                command = [
                    axes[self.Axis.JOINT_1] * self.config.max_qd[0],
                    axes[self.Axis.JOINT_2] * self.config.max_qd[1],
                    axes[self.Axis.JOINT_3] * self.config.max_qd[2],
                    axes[self.Axis.JOINT_4] * self.config.max_qd[3],
                    0.0,
                    0.0,
                ]
            elif self.movement_mode == self.MovementMode.ANGULAR:
                command = [
                    0.0,
                    0.0,
                    axes[self.Axis.JOINT_3] * self.config.max_qd[2],
                    axes[self.Axis.JOINT_4] * self.config.max_qd[3],
                    axes[self.Axis.JOINT_5] * self.config.max_qd[4],
                    axes[self.Axis.JOINT_6] * self.config.max_qd[5],
                ]
            self.command_sender.send_joint_command(command)

    def add_gui(self, gui):
        self.gui = gui

    def _update_gui(self, axes, buttons):
        if self.gui is not None:
            self.gui.update(axes, buttons, self.space_mode, self.movement_mode, self.frame)

    def _send_preset_request(self, preset_name: str):
        if preset_name not in MANIP_PRESET_DATABASE:
            print(f"Manip preset {preset_name} is not defined")
            return
        target_q = MANIP_PRESET_DATABASE[preset_name]
        self.command_sender.send_preset_command(target_q)

    def _control_gripper(self, gripper_axis: float):
        gripper_force_cmd = self.gripper_controller.step(gripper_axis)
        self.command_sender.send_gripper_command(gripper_force_cmd)


if __name__ == "__main__":
    print("Starting joystick control")
    joystick_control = JoystickControl()
    joystick_control.run()
    print("Joystick control finished")
