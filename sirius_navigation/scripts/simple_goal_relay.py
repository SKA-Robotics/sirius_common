#!/usr/bin/env python3
"""Relay move_base_simple/goal PoseStamped messages to nav2 NavigateToPose action."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class SimpleGoalRelay(Node):

    def __init__(self):
        super().__init__('simple_goal_relay')

        self._action_client = ActionClient(self, NavigateToPose,
                                           'navigate_to_pose')
        self._current_goal: ClientGoalHandle = None

        self.create_subscription(PoseStamped, 'move_base_simple/goal',
                                 self._goal_callback, 10)

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready.')

    def _goal_callback(self, msg: PoseStamped):
        if self._current_goal is not None:
            self.get_logger().info('Canceling previous goal')
            self._current_goal.cancel_goal()
            self._current_goal = None

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f'Sending goal: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        handle: ClientGoalHandle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self._current_goal = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self._current_goal = None
        result = future.result()
        self.get_logger().info(f'Navigation completed: status={result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
