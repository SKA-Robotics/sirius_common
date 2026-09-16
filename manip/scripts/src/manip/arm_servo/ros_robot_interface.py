import threading
import time
from typing import Tuple, Optional, List

import numpy as np
from sensor_msgs.msg import JointState


class RosRobotInterface:

    def __init__(self, joint_names: List[str], state_topic: str,
                 command_topic: str, node):
        self.joint_state_lock = threading.Lock()
        self.joint_names = joint_names
        self.state_topic = state_topic
        self.received_joints: set = set()
        self.q = np.zeros(len(joint_names))
        self.qd = np.zeros(len(joint_names))
        self._node = node

        node.create_subscription(JointState, state_topic,
                                 self._joint_state_callback, 10)
        self._pub = node.create_publisher(JointState, command_topic, 10)

    def _joint_state_callback(self, msg: JointState):
        if len(msg.velocity) != len(msg.position):
            msg.velocity = [0.0] * len(msg.position)

        with self.joint_state_lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.received_joints.add(name)
                    idx = self.joint_names.index(name)
                    self.q[idx] = msg.position[i]
                    self.qd[idx] = msg.velocity[i]

    def state_received(self) -> bool:
        return all(name in self.received_joints for name in self.joint_names)

    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.state_received():
            print(f'Waiting for robot joint state on topic {self.state_topic}')
            while not self.state_received():
                time.sleep(0.3)
            print('Robot joint state received')
        with self.joint_state_lock:
            return self.q.copy(), self.qd.copy()

    def set_joint_state(self, q: Optional[np.ndarray] = None,
                        qd: Optional[np.ndarray] = None) -> None:
        if q is None and qd is None:
            return
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = self.joint_names
        if q is not None:
            msg.position = q.tolist()
        if qd is not None:
            msg.velocity = qd.tolist()
        self._pub.publish(msg)
