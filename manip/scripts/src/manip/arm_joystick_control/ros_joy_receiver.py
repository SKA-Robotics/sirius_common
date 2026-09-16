from typing import Callable, List

from joystick_control.msg import Gamepad


class RosJoyReceiver:

    def __init__(self, topic: str, node):
        self.callback = None
        node.create_subscription(Gamepad, topic, self._joy_callback, 10)

    def _joy_callback(self, msg: Gamepad):
        if self.callback is not None:
            self.callback(msg.axes, msg.buttons)

    def register_callback(self, callback: Callable[[List[float], List[int]], None]):
        self.callback = callback
