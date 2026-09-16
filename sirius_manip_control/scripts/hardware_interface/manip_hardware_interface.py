from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState


JOINT_NAMES = ['base_cyl', 'cyl_arm1', 'arm1_arm2', 'arm2_arm3', 'arm3_tool']


class ManipHardwareInterface(ABC, Node):

    def __init__(self, node_name: str = 'manip_interface') -> None:
        Node.__init__(self, node_name)

        self.declare_parameter('command_topic', '/manip_interface/command')
        self.declare_parameter('state_topic', '/manip_interface/state')
        for joint in JOINT_NAMES:
            self.declare_parameter(f'offset_{joint}', 0.0)

        self.offsets = {
            joint: self.get_parameter(f'offset_{joint}').value
            for joint in JOINT_NAMES
        }
        self.add_on_set_parameters_callback(self._on_parameter_change)

        manip_command_topic = self.get_parameter('command_topic').value
        manip_state_topic = self.get_parameter('state_topic').value

        self._manip_state_pub = self.create_publisher(
            JointState, manip_state_topic, 10)
        self.create_subscription(JointState, manip_command_topic,
                                 self._send_command, 10)

        self._initialize_hardware_connection()

    def _on_parameter_change(self, params):
        for param in params:
            for joint in JOINT_NAMES:
                if param.name == f'offset_{joint}':
                    self.offsets[joint] = param.value
        return SetParametersResult(successful=True)

    def _publish_state(self, joint_state: JointState):
        self._manip_state_pub.publish(
            self._transform_jointstate(joint_state, 1))

    def _send_command(self, joint_state: JointState):
        self._send_hardware_command(
            self._transform_jointstate(joint_state, -1))

    def _transform_jointstate(self, joint_state: JointState,
                               multiplier: int) -> JointState:
        out = JointState()
        out.header = joint_state.header
        if not joint_state.position:
            out.name = joint_state.name
            out.velocity = joint_state.velocity
            out.effort = joint_state.effort
            return out

        for i, pos in enumerate(joint_state.position):
            name = joint_state.name[i]
            if name not in self.offsets:
                self.get_logger().warn(f'Missing offset for joint: {name}')
                continue
            out.name.append(name)
            out.position.append(pos + multiplier * self.offsets[name])
        return out

    @abstractmethod
    def _initialize_hardware_connection(self):
        pass

    @abstractmethod
    def _send_hardware_command(self, joint_state: JointState):
        pass
