#!/usr/bin/env python3
"""
Publishes a diagnostic summary of the Sirius rover's drive state.

Subscribes to:
  - joy_multiplexer/selected_output (std_msgs/String)
  - relaxing_middleware/state       (std_msgs/String)

Publishes:
  - robot_status (diagnostic_msgs/DiagnosticArray)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SiriusStatus(Node):

    def __init__(self):
        super().__init__('sirius_status')

        self._mode = 'UNKNOWN'       # UNKNOWN | MANUAL | AUTONOMOUS
        self._drive_state = 'Idle'   # Idle | Running | Breaking | Relaxing | Locked

        self.create_subscription(
            String, 'joy_multiplexer/selected_output',
            self._joy_cb, 10)
        self.create_subscription(
            String, 'relaxing_middleware/state',
            self._relaxing_cb, 10)

        self._pub = self.create_publisher(DiagnosticArray, 'robot_status', 10)
        self.create_timer(0.5, self._publish)

    def _joy_cb(self, msg: String):
        if msg.data == '__none':
            self._mode = 'UNKNOWN'
        elif msg.data in ('joy_diff_drive', 'joy_5dof_manipulator'):
            self._mode = 'MANUAL'
        elif msg.data == 'autonomy':
            self._mode = 'AUTONOMOUS'
        self._publish()

    def _relaxing_cb(self, msg: String):
        self._drive_state = msg.data
        self._publish()

    def _publish(self, *_):
        in_motion = self._drive_state in ('Running', 'Breaking', 'Relaxing')
        drives_on = self._drive_state in ('Running', 'Breaking', 'Relaxing', 'Locked')

        status = DiagnosticStatus()
        status.name = 'sirius/drive'
        status.level = (DiagnosticStatus.OK
                        if drives_on else DiagnosticStatus.WARN)
        status.message = self._drive_state
        status.values = [
            KeyValue(key='mode', value=self._mode),
            KeyValue(key='drives_powered', value=str(drives_on)),
            KeyValue(key='in_motion', value=str(in_motion)),
            KeyValue(key='drive_state', value=self._drive_state),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SiriusStatus()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
