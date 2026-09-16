import time

from rclpy.node import Node
from sensor_msgs.msg import JointState

from manip_interface import ManipInterface, ManipParams
from ik import ManipJointState


class ROSManipInterface(ManipInterface):

    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        node.get_logger().info('Initializing ROSManipInterface')
        time.sleep(2)
        self.jointstate = None
        self._params = self._load_ros_params()
        self._initialize_topics()

    def _initialize_topics(self):
        if not self._node.has_parameter('jointstate_topic'):
            self._node.declare_parameter('jointstate_topic',
                                         '/manip_interface/state')
        if not self._node.has_parameter('command_topic'):
            self._node.declare_parameter('command_topic',
                                         '/manip_interface/command')

        jointstate_topic = self._node.get_parameter('jointstate_topic').value
        command_topic = self._node.get_parameter('command_topic').value

        self.subscriber = self._node.create_subscription(
            JointState, jointstate_topic, self._update_jointstate, 10)
        self.publisher = self._node.create_publisher(
            JointState, command_topic, 10)

    def _load_ros_params(self) -> ManipParams:
        if not self._node.has_parameter('control_modes'):
            self._node.declare_parameter('control_modes',
                                         rclpy.Parameter.Type.STRING)
        if not self._node.has_parameter('links'):
            self._node.declare_parameter('links', rclpy.Parameter.Type.STRING)

        import rclpy
        import yaml
        raw_modes = self._node.get_parameter('control_modes').value
        raw_links = self._node.get_parameter('links').value
        if isinstance(raw_modes, str):
            raw_modes = yaml.safe_load(raw_modes)
        if isinstance(raw_links, str):
            raw_links = yaml.safe_load(raw_links)

        return ManipParams.from_dict({
            'links': raw_links,
            'control_modes': raw_modes,
        })

    def get_jointstate(self) -> ManipJointState:
        while self.jointstate is None:
            self._node.get_logger().warn_once(
                'Waiting for manip joint state...')
            time.sleep(0.1)
        return self.jointstate

    def set_jointstate(self, jointstate: ManipJointState):
        msg = JointStateConverter.manip_to_ros(
            jointstate, self._params.joint_names(),
            self._node.get_clock().now().to_msg())
        self.publisher.publish(msg)

    def _update_jointstate(self, msg: JointState):
        self.jointstate = JointStateConverter.ros_to_manip(
            msg, self._params.joint_names())

    def sleep(self, duration: float):
        time.sleep(duration)

    def get_manip_params(self) -> ManipParams:
        return self._params


class JointStateConverter:

    @staticmethod
    def manip_to_ros(jointstate: ManipJointState, names, stamp) -> JointState:
        assert len(jointstate.position) == len(names)
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = list(jointstate.position)
        return msg

    @staticmethod
    def ros_to_manip(jointstate: JointState, names) -> ManipJointState:
        filtered = JointStateConverter.filter_ros(jointstate, names)
        return ManipJointState.from_list(filtered.position)

    @staticmethod
    def filter_ros(jointstate: JointState, names) -> JointState:
        filtered = JointState()
        filtered.header = jointstate.header
        for name in names:
            idx = jointstate.name.index(name)
            filtered.name.append(name)
            filtered.position.append(jointstate.position[idx])
        return filtered
