from typing import Tuple, Optional, List
import numpy as np
import roboticstoolbox as rtb
from sensor_msgs.msg import JointState
import threading
import rospy
import time

class RosRobotInterface:
    def __init__(self, joint_names: List[str], state_topic: str, command_topic: str):
        self.joint_state_lock = threading.Lock()
        self.joint_names = joint_names
        self.state_topic = state_topic
        self.received_joints = set()
        self.q = np.zeros(len(joint_names))
        self.qd = np.zeros(len(joint_names))
        self.joint_state_sub = rospy.Subscriber(state_topic, JointState, self.joint_state_callback, queue_size=10)
        self.joint_state_pub = rospy.Publisher(command_topic, JointState, queue_size=10)


    def joint_state_callback(self, msg: JointState):
        if len(msg.velocity) != len(msg.position):
            msg.velocity = [0.0] * len(msg.position)
        
        with self.joint_state_lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.received_joints.add(name)
                    index = self.joint_names.index(name)
                    self.q[index] = msg.position[i]
                    self.qd[index] = msg.velocity[i]



    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.state_received():
            print(f"Waiting for robot joint state on topic {self.state_topic}")
            while not self.state_received():
                time.sleep(0.3)
            print("Robot joint state received")
        with self.joint_state_lock:
            return self.q, self.qd

    def set_joint_state(self, q: Optional[np.ndarray] = None, qd: Optional[np.ndarray] = None) -> None:
        if qd is None and q is None:
            return

        if qd is None:
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = q.tolist()
            self.joint_state_pub.publish(msg)

        if q is None:
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.velocity = qd.tolist()
            self.joint_state_pub.publish(msg)
    
    def state_received(self):
        return all([name in self.received_joints for name in self.joint_names])
