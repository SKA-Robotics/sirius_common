import rospy
from typing import Callable, List
from joystick_control.msg import Gamepad


class RosJoyReceiver:

    def __init__(self, topic: str):
        rospy.Subscriber(
            topic, Gamepad, self.joy_callback, queue_size=10)
        self.callback = None

    def joy_callback(self, msg: Gamepad):
        if self.callback is not None:
            self.callback(msg.axes, msg.buttons)

    def register_callback(self, callback: Callable[[List[float], List[int]],
                                                   None]):
        self.callback = callback
