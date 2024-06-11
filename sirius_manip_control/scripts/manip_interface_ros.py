import rospy

from sensor_msgs.msg import JointState

# from sirius_msgs.msg import JointState as SiriusJointState
from manip_interface import ManipInterface, ManipParams
from ik import ManipJointState


class ROSManipInterface(ManipInterface):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Initializg ROSManipInterface")
        self.jointstate = None
        self._params = self._load_ROSparams()
        self._initialize_topics()

    def _initialize_topics(self):
        self.jointstate_topic = rospy.get_param(
            "~jointstate_topic", "/manip_interface/state"
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/manip_interface/command"
        )
        self.subscriber = rospy.Subscriber(
            self.jointstate_topic, JointState, self._update_jointstate
        )
        self.publisher = rospy.Publisher(self.command_topic, JointState, queue_size=10)

    def _load_ROSparams(self):
        mode_params = rospy.get_param("~control_modes")
        link_params = rospy.get_param("~links")
        params_dict = {"links": link_params, "control_modes": mode_params}
        return ManipParams.from_dict(params_dict)

    def get_jointstate(self) -> ManipJointState:
        while self.jointstate is None:
            rospy.logwarn_once(
                "Trying to get jointstate of manip, but it has not been received yet. Waiting."
            )
        return self.jointstate

    def set_jointstate(self, jointstate: ManipJointState):
        self.jointstate = jointstate
        jointstate_cmd = JointStateConverter.manip_to_ros(
            jointstate, self._params.joint_names()
        )

        self.publisher.publish(jointstate_cmd)

    def _update_jointstate(self, msg):
        self.actual_jointstate = JointStateConverter.ros_to_manip(
            msg, self._params.joint_names()
        )

        if self.jointstate is None:
            self.jointstate = self.actual_jointstate

    def sleep(self, time):
        rospy.sleep(rospy.Duration(time))

    def get_manip_params(self) -> ManipParams:
        return self._params


class JointStateConverter:
    def manip_to_ros(joinstate: ManipJointState, names) -> JointState:
        assert len(joinstate.position) == len(names)
        result = JointState()
        result.header.stamp = rospy.Time.now()
        result.name = names
        result.position = joinstate.position
        return result

    def ros_to_manip(jointstate: JointState, names) -> ManipJointState:
        filtered_jointstate = JointStateConverter.filter_ros(jointstate, names)
        return ManipJointState.from_list(filtered_jointstate.position)

    def filter_ros(jointstate: JointState, names):
        filtered_jointstate = JointState()
        filtered_jointstate.header = jointstate.header
        for name in names:
            index = jointstate.name.index(name)
            filtered_jointstate.name.append(name)
            filtered_jointstate.position.append(jointstate.position[index])
        return filtered_jointstate
