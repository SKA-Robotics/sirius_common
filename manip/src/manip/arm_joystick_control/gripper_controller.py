import time
from manip.arm_joystick_control.utils import Debouncing


class GripperController:
    def __init__(self):
        self.debouncer = Debouncing()
        self.gripper_cmd = 0
        self.pressed_time = time.time()
        self.hold_to_release_time = 0.6
        self.release_cmd_value = -0.8692137
        self.gripper_cmd_increment = 0.1
    
    def step(self, value: float) -> float:
        self.debouncer.update(value)
        if self.debouncer.is_falling_edge:
            if time.time() - self.pressed_time < self.hold_to_release_time:
                # When released quickly after pressing, increment the gripper command
                if self.gripper_cmd < 1:
                    self.gripper_cmd += self.gripper_cmd_increment
                if self.gripper_cmd > 1:
                    self.gripper_cmd = 1
            else:
                # When released after the gripper is opened, reset the command to zero
                self.gripper_cmd = 0
        if self.debouncer.state == False:
            self.pressed_time = time.time()
        else:
            if time.time() - self.pressed_time > self.hold_to_release_time:
                self.gripper_cmd = self.release_cmd_value
        return self.gripper_cmd