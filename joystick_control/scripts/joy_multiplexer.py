#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'utils'))

from functools import partial
from threading import Lock

import rclpy
from rclpy.node import Node

from joystick_control.msg import Topic, TopicArray, Gamepad
from joystick_control.srv import GetTopic, GetTopicArray, SendTopic

from utils.translate_joystick import JoystickTranslator
from utils.debouncing import Debouncing


class JoystickMultiplexer(Node):

    def __init__(self) -> None:
        super().__init__('joy_multiplexer')

        self.declare_parameter('joystick_type', 'STANDARD')
        self.declare_parameter('steering_modes', rclpy.Parameter.Type.STRING)

        import yaml
        raw = self.get_parameter('steering_modes').value
        if isinstance(raw, str):
            self.STEERING_MODES = yaml.safe_load(raw)
        else:
            self.STEERING_MODES = raw or {}

        self.active_joystick = '__none'
        self.joystick_subscribers = {}
        self.active_output = '__none'
        self.output_publishers = {
            name: self.create_publisher(Gamepad, name, 10)
            for name in self.STEERING_MODES.keys()
            if name != 'emergency'
        }

        self.joy_list_publisher = self.create_publisher(
            TopicArray, '~/joy_list', 10)
        self.selected_joy_publisher = self.create_publisher(
            Topic, '~/selected_joy', 10)
        self.selected_output_publisher = self.create_publisher(
            Topic, '~/selected_output', 10)

        self.create_service(GetTopicArray, '~/get_joy_list',
                            self._get_joy_list)
        self.create_service(SendTopic, '~/add_joy', self._add_joy)
        self.create_service(SendTopic, '~/remove_joy', self._remove_joy)
        self.create_service(SendTopic, '~/select_joy', self._select_joy)
        self.create_service(SendTopic, '~/select_output', self._select_output)
        self.create_service(GetTopic, '~/get_selected_joy',
                            self._get_selected_joy)
        self.create_service(GetTopic, '~/get_selected_output',
                            self._get_selected_output)

        self.prev_inputs = {}
        self.service_lock = Lock()
        self.translator = JoystickTranslator(self)

    def _joy_subscriber_callback(self, data: Gamepad, topic_name=''):
        inputs = self.translator.translate(data)
        debounce = Debouncing(inputs, self.prev_inputs[topic_name])
        self.prev_inputs[topic_name] = inputs

        with self.service_lock:
            # emergency stop
            if debounce.is_leading_edge(
                    self.STEERING_MODES['emergency']['button']):
                self.get_logger().warn('emergency stop')
                self._set_joystick('__none')
                self._set_output('__none')
                return

            # controller mode selection
            for name, config in self.STEERING_MODES.items():
                if name == 'emergency':
                    continue
                if debounce.is_leading_edge(config['button']):
                    self.get_logger().warn(
                        f"enabling {config['topic']} with joystick {topic_name}"
                    )
                    self._set_joystick(topic_name)
                    self._set_output(config['topic'])

            # forward input to active output
            if (topic_name == self.active_joystick
                    and self.active_output != '__none'):
                self.output_publishers[self.active_output].publish(data)

    def _set_joystick(self, topic_name):
        self.active_joystick = topic_name
        msg = Topic()
        msg.name = topic_name
        self.selected_joy_publisher.publish(msg)

    def _set_output(self, topic_name):
        self.active_output = topic_name
        msg = Topic()
        msg.name = topic_name
        self.selected_output_publisher.publish(msg)

    def _publish_joy_list_update(self):
        msg = TopicArray()
        msg.array = [
            self._make_topic(name)
            for name in self.joystick_subscribers.keys()
        ]
        self.joy_list_publisher.publish(msg)

    def _make_topic(self, name: str) -> Topic:
        t = Topic()
        t.name = name
        return t

    def _get_selected_joy(self, request, response):
        with self.service_lock:
            response.topic = self._make_topic(self.active_joystick)
        return response

    def _get_selected_output(self, request, response):
        with self.service_lock:
            response.topic = self._make_topic(self.active_output)
        return response

    def _select_output(self, request, response):
        topic_name = request.topic.name
        self.get_logger().info(f'Selecting output {topic_name}')
        with self.service_lock:
            if topic_name == '__none':
                self._set_joystick('__none')
                self._set_output('__none')
                response.success = True
                return response
            if topic_name not in self.output_publishers:
                response.success = False
                return response
            self._set_output(topic_name)
            response.success = True
        return response

    def _select_joy(self, request, response):
        topic_name = request.topic.name
        self.get_logger().info(f'Selecting joystick {topic_name}')
        with self.service_lock:
            if topic_name == '__none':
                self._set_joystick('__none')
                self._set_output('__none')
                response.success = True
                return response
            if topic_name not in self.joystick_subscribers:
                response.success = False
                return response
            self._set_joystick(topic_name)
            response.success = True
        return response

    def _add_joy(self, request, response):
        topic_name = request.topic.name
        self.get_logger().info(f'Adding {topic_name}')
        with self.service_lock:
            if (topic_name in self.joystick_subscribers
                    or topic_name == '__none'):
                response.success = False
                return response

            self.prev_inputs[topic_name] = {
                key: 0
                for key in self.translator.BUTTONS_ID.keys()
            }
            self.joystick_subscribers[topic_name] = self.create_subscription(
                Gamepad,
                topic_name,
                partial(self._joy_subscriber_callback,
                        topic_name=topic_name),
                10,
            )
            self._publish_joy_list_update()
            response.success = True
        return response

    def _remove_joy(self, request, response):
        topic_name = request.topic.name
        self.get_logger().info(f'Removing {topic_name}')
        with self.service_lock:
            if topic_name not in self.joystick_subscribers:
                response.success = False
                return response

            if topic_name == self.active_joystick:
                self._set_joystick('__none')

            del self.prev_inputs[topic_name]
            self.destroy_subscription(
                self.joystick_subscribers[topic_name])
            del self.joystick_subscribers[topic_name]

            self._publish_joy_list_update()
            response.success = True
        return response

    def _get_joy_list(self, request, response):
        with self.service_lock:
            msg = TopicArray()
            msg.array = [
                self._make_topic(name)
                for name in self.joystick_subscribers.keys()
            ]
            response.topics = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = JoystickMultiplexer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()