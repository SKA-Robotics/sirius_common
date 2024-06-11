#!/usr/bin/python3
import roslib

roslib.load_manifest("joystick_control")

import rospy
from threading import Lock
from functools import partial

from joystick_control.msg import Topic, TopicArray, Gamepad
from joystick_control.srv import GetTopic, GetTopicArray, SendTopic

from utils.translate_joystick import JoystickTranslator
from utils.debouncing import Debouncing


class JoystickMultiplexer:

    def __init__(self) -> None:
        rospy.init_node("joy_multiplexer")

        self.STEERING_MODES = rospy.get_param("~steering_modes", None)

        self.active_joystick = "__none"
        self.joystick_subscribers = {}
        self.active_output = "__none"
        self.output_publishers = {
            name: rospy.Publisher(name, Gamepad, queue_size=10)
            for name in self.STEERING_MODES.keys() if name != "emergency"
        }

        self.joy_list_publisher = rospy.Publisher("~joy_list",
                                                  TopicArray,
                                                  queue_size=10)
        rospy.Service("~get_joy_list", GetTopicArray, self._get_joy_list)
        rospy.Service("~add_joy", SendTopic, self._add_joy)
        rospy.Service("~remove_joy", SendTopic, self._remove_joy)

        self.selected_joy_publisher = rospy.Publisher("~selected_joy",
                                                      Topic,
                                                      queue_size=10)
        self.selected_output_publisher = rospy.Publisher("~selected_output",
                                                         Topic,
                                                         queue_size=10)
        rospy.Service("~select_joy", SendTopic, self._select_joy)
        rospy.Service("~select_output", SendTopic, self._select_output)
        rospy.Service("~get_selected_joy", GetTopic, self._get_selected_joy)
        rospy.Service("~get_selected_output", GetTopic,
                      self._get_selected_output)

        # key debouncing
        self.prev_inputs = {}

        # required, because services run on different threads and have to be thread-safe
        self.service_lock = Lock()

        self.translator = JoystickTranslator()

    def run(self) -> None:
        rospy.spin()

    def _joy_subscriber_callback(self, data: Gamepad, topic_name=""):
        inputs = self.translator.translate(data)
        debounce = Debouncing(inputs, self.prev_inputs[topic_name])
        self.prev_inputs[topic_name] = inputs

        with self.service_lock:
            # emergency stop
            if debounce.is_leading_edge(
                    self.STEERING_MODES["emergency"]["button"]):
                rospy.logwarn("emergency stop")
                self.set_joystick("__none")
                self.set_output("__none")
                return

            # controller mode selection
            for config in self.STEERING_MODES.values():
                if debounce.is_leading_edge(config["button"]):
                    rospy.logwarn(
                        f"enabling {config['topic']} with joystick {topic_name}"
                    )
                    self.set_joystick(topic_name)
                    self.set_output(config["topic"])

            # input handling
            if topic_name == self.active_joystick and self.active_output != "__none":
                self.output_publishers[self.active_output].publish(data)

    def set_joystick(self, topic_name):
        self.active_joystick = topic_name
        self.selected_joy_publisher.publish(Topic(topic_name))

    def set_output(self, topic_name):
        self.active_output = topic_name
        self.selected_output_publisher.publish(Topic(topic_name))

    def publish_joy_list_update(self):
        self.joy_list_publisher.publish([
            Topic(topic_name)
            for topic_name in self.joystick_subscribers.keys()
        ])

    def _get_selected_joy(self, req):
        with self.service_lock:
            return Topic(self.active_joystick)

    def _get_selected_output(self, req):
        with self.service_lock:
            return Topic(self.active_output)

    def _select_output(self, req):
        topic_name = req.topic.name
        rospy.loginfo(f"Selecting output {topic_name}")
        with self.service_lock:
            # allow empty name to disable control
            if topic_name == "__none":
                self.set_joystick("__none")
                self.set_output("__none")
                return True

            # only allow topics that have been added already
            if topic_name not in self.output_publishers:
                return False

            self.set_output(topic_name)
            return True

    def _select_joy(self, req):
        topic_name = req.topic.name
        rospy.loginfo(f"Selecting joystick {topic_name}")
        with self.service_lock:
            # allow empty name to disable control
            if topic_name == "__none":
                self.set_joystick("__none")
                self.set_output("__none")
                return True

            # only allow topics that have been added already
            if topic_name not in self.joystick_subscribers:
                return False

            self.set_joystick(topic_name)
            return True

    def _add_joy(self, req):
        topic_name = req.topic.name
        rospy.loginfo(f"Adding {topic_name}")
        with self.service_lock:
            # prevent topic collisions and __none, which is reserved
            if topic_name in self.joystick_subscribers or topic_name == "__none":
                return False

            self.prev_inputs[topic_name] = {
                key: 0
                for key in self.translator.BUTTONS_ID.keys()
            }
            self.joystick_subscribers[topic_name] = rospy.Subscriber(
                topic_name,
                Gamepad,
                partial(self._joy_subscriber_callback, topic_name=topic_name),
            )

            self.publish_joy_list_update()
            return True

    def _remove_joy(self, req):
        topic_name = req.topic.name
        rospy.loginfo(f"Removing {topic_name}")
        with self.service_lock:
            # don't remove topics that don't exist
            if topic_name not in self.joystick_subscribers:
                return False

            if topic_name == self.active_joystick:
                self.set_joystick("__none")

            del self.prev_inputs[topic_name]
            self.joystick_subscribers[topic_name].unregister()
            del self.joystick_subscribers[topic_name]

            self.publish_joy_list_update()
            return True

    def _get_joy_list(self, req):
        with self.service_lock:
            # create a copy to avoid race conditions
            return {
                "topics":
                TopicArray([
                    Topic(topic_name)
                    for topic_name in self.joystick_subscribers.keys()
                ])
            }


if __name__ == "__main__":
    try:
        JoystickMultiplexer().run()
    except rospy.ROSInterruptException:
        pass
