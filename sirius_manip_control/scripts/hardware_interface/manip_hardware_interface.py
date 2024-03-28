import rospy

from sensor_msgs.msg import JointState

from dynamic_reconfigure.server import Server
from manip_control.cfg import ManipInterfaceConfig

from abc import ABC, abstractmethod


class ManipHardwareInterface(ABC):
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("manip_interface")
        self._load_params()
        self._initialize_ros_topics()
        self._initialize_hardware_connection()

        self.config_server = Server(ManipInterfaceConfig, self._dynamic_reconfigure)

    def run(self):
        self._pre_run()
        rospy.spin()

    def _dynamic_reconfigure(self, config, level):
        self.offsets = {
            "base_cyl": config["offset_base_cyl"],
            "cyl_arm1": config["offset_cyl_arm1"],
            "arm1_arm2": config["offset_arm1_arm2"],
            "arm2_arm3": config["offset_arm2_arm3"],
            "arm3_tool": config["offset_arm3_tool"],
        }

        return config

    def _load_params(self):
        self.manip_command_topic = rospy.get_param(
            "~command_topic", "/manip_interface/command"
        )
        self.manip_state_topic = rospy.get_param(
            "~state_topic", "/manip_interface/state"
        )

    def _initialize_ros_topics(self):
        self.manip_state = rospy.Publisher(
            self.manip_state_topic, JointState, queue_size=10
        )
        self.manip_command = rospy.Subscriber(
            self.manip_command_topic, JointState, self._send_command
        )

    def _publish_state(self, joint_state: JointState):
        self.manip_state.publish(self._transform_jointstate(joint_state, 1))

    def _send_command(self, joint_state: JointState):
        self._send_hardware_command(self._transform_jointstate(joint_state, -1))

    def _transform_jointstate(self, joint_state: JointState, multiplier):
        new_jointstate = JointState()
        for index, position in enumerate(joint_state.position):
            name = joint_state.name[index]
            if name not in self.offsets.keys():
                rospy.logwarn(f"Missing offset for joint: {name}")
                continue

            new_jointstate.name.append(name)
            new_jointstate.position.append(position + (multiplier * self.offsets[name]))

        new_jointstate.header = joint_state.header
        if len(joint_state.position) == 0:
            new_jointstate.name = joint_state.name
            new_jointstate.velocity = joint_state.velocity
            new_jointstate.effort = joint_state.effort

        return new_jointstate

    @abstractmethod
    def _initialize_hardware_connection(self):
        pass

    @abstractmethod
    def _send_hardware_command(self, joint_state: JointState):
        pass

    def _pre_run(self):
        pass
