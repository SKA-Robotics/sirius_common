from enum import Enum
from typing import List, Dict
from manip.arm_joystick_control.ros_joy_receiver import RosJoyReceiver
from manip.arm_joystick_control.ros_command_sender import RosCommandSender
from manip.arm_joystick_control.gripper_controller import GripperController
# from manip.arm_joystick_control.gather_joystick_presets import RosJointStateReceiver, write_json
from manip.arm_joystick_control.utils import max_abs, trig_to_axis, JoystickTranslator
from manip.manip_config import ManipConfig, DEFAULT_CONFIG
import time
import rospy
import json
import os

# MANIP_PRESET_DATABASE = {
#     "ik_ready": [0.0, -0.5, 1.85, -1.53, 0.96, -0.06],
#     "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#     "ground": [0.0, 1.0, 1.462, -1.618, 0.6327, 0.0],
#     # "side_box": [-2.3279, 0.3627, 1.4217, -1.2624, 1.0883, 0.0651]
#     # "side_box": [-2.3655, 0.3531, 1.7056, -1.2195, 0.9403, -0.8759]
#     "side_box": [-2.4366326077570983, 0.2162912910918754, 1.6567951247157353, -1.2455923997631406, 1.1382137446111458, -0.8835729338221293]

# }


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
        LEFT_CROSS = 5
        UP_CROSS = 6
        DOWN_CROSS = 7
        RIGHT_CROSS = 8
        # QUICK_CHANGE_PRESET = 9

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
        self.command_sender = RosCommandSender(config.twist_topic,
                                               config.joint_topic,
                                               config.gripper_cmd_topic,
                                               config.preset_request_topic)
        self.joy_receiver = RosJoyReceiver(config.joy_topic)
        self.joint_states_receiver = RosJointStateReceiver()
        # self.quick_change_mode = False
        # self.last_preset_button_state = False

    def run(self):
        self.joy_receiver.register_callback(self.receive_command)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    # def _quick_change_preset(self, button: str):
    #     joint_states = jointReceiver.get_current_positions()
    #     data = self._read_json()
    #     if not data:
    #         return

    #     preset_name = data["key_mappings"].get(button)

    #     if preset_name and "presets" in data:
    #         data["presets"][preset_name] = joint_states

    #         write_json(data)

    def receive_command(self, raw_axes: List[float], raw_buttons: List[int]):
        input = JoystickTranslator().translate({
            "axes": raw_axes,
            "buttons": raw_buttons
        })
        buttons = self._process_buttons(input)
        self._handle_buttons(buttons)

        # if buttons[self.Button.QUICK_CHANGE_PRESET]:
        #     if not self.last_preset_button_state:
        #         self.quick_change_mode = not self.quick_change_mode
        #         print(f"Quick change mode: {self.quick_change_mode}")
        #     self.last_preset_button_state = True
        # else:
        #     self.last_preset_button_state = False

        if buttons[self.Button.LEFT_CROSS]:
            self._send_preset_request("LEFT_CROSS")
        elif buttons[self.Button.UP_CROSS]:
            self._send_preset_request("UP_CROSS")
        elif buttons[self.Button.DOWN_CROSS]:
            self._send_preset_request("DOWN_CROSS")
        elif buttons[self.Button.RIGHT_CROSS]:
            self._send_preset_request("RIGHT_CROSS")

        self._update_gui(raw_axes, raw_buttons)
        axes = self._process_axes(input)
        self._publish_command(axes)
        self._control_gripper(axes[self.Axis.GRIPPER])

    def _process_axes(self, input: Dict[str, float]) -> Dict[Axis, float]:
        return {
            self.Axis.LINEAR_X:
            -input["left_stick_vertical"],
            self.Axis.LINEAR_Y:
            max_abs(-input["left_stick_horizontal"],
                    -input["right_stick_horizontal"]),
            self.Axis.LINEAR_Z:
            -input["right_stick_vertical"],
            self.Axis.ANGULAR_X:
            input["left_stick_horizontal"],
            self.Axis.ANGULAR_Y:
            max_abs(input["left_stick_vertical"],
                    input["right_stick_vertical"]),
            self.Axis.ANGULAR_Z:
            -input["right_stick_horizontal"],
            self.Axis.GRIPPER:
            max_abs(input["left_trigger"], input["right_trigger"]),
            self.Axis.JOINT_1:
            -input["left_stick_horizontal"],
            self.Axis.JOINT_2:
            -input["left_stick_vertical"],
            self.Axis.JOINT_3:
            -input["right_stick_vertical"],
            self.Axis.JOINT_4:
            input["right_stick_horizontal"],
            self.Axis.JOINT_5:
            -input["left_stick_vertical"],
            self.Axis.JOINT_6:
            input["left_stick_horizontal"],
        }

    def _process_buttons(self, input: Dict[str, float]) -> Dict[Button, bool]:
        return {
            self.Button.SET_FRAME_BASE:
            input["start_button"]
            and (input["left_bumper"] or input["right_bumper"]),
            self.Button.SET_FRAME_TOOL:
            input["start_button"]
            and not (input["left_bumper"] or input["right_bumper"]),
            self.Button.SET_JOINT_MODE:
            input["back_button"],
            self.Button.CHANGE_MOVEMENT_MODE:
            input["left_bumper"] or input["right_bumper"],
            self.Button.LEFT_CROSS:
            input["left_cross"],
            self.Button.UP_CROSS:
            input["up_cross"],
            self.Button.DOWN_CROSS:
            input["down_cross"],
            self.Button.RIGHT_CROSS:
            input["right_cross"],
            self.Button.QUICK_CHANGE_PRESET:
            input["X_button"] and input["B_button"]
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
            self.gui.update(axes, buttons, self.space_mode, self.movement_mode,
                            self.frame)

    def _read_json(self, filename='presets.json'):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        full_path = os.path.join(current_dir, filename)

        try:
            with open(full_path, 'r') as file:
                return json.load(file)
                # return file_data.get(preset_name, None)
        except FileNotFoundError:
            print("Error: File 'presets.json' does not exist yet.")
            return None
        except json.JSONDecodeError:
            print("Error: 'presets.json' is corrupted or empty.")
            return None

    def _send_preset_request(self, button: str):
        data = self._read_json()
        preset_name = data["key_mappings"].get(button)
        target_q = data["presets"].get(preset_name)
        if not target_q:
            print(f"Manip preset {preset_name} is not defined")
            return

        self.command_sender.send_preset_command(target_q)

    def _control_gripper(self, gripper_axis: float):
        gripper_force_cmd = self.gripper_controller.step(gripper_axis)
        self.command_sender.send_gripper_command(gripper_force_cmd)


if __name__ == "__main__":
    print("Starting joystick control")
    joystick_control = JoystickControl()
    joystick_control.run()
    print("Joystick control finished")
