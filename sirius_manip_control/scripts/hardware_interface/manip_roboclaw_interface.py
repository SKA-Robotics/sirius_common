#!/usr/bin/python3
import rospy

from functools import partial

from sensor_msgs.msg import JointState
from sirius_msgs.msg import JointState as SiriusJointState

from hardware_interface.manip_hardware_interface import (
    ManipHardwareInterface,
)


class ManipRoboclawInterface(ManipHardwareInterface):
    def __init__(self):
        super().__init__()

    def _initialize_hardware_connection(self):
        self.command_topics = rospy.get_param("~command_topics", {})
        self.feedback_topics = rospy.get_param("~feedback_topics", {})
        self._rate = rospy.get_param("~rate", 10)
        self.joint_positions = {key: 0.0 for key in self.feedback_topics.keys()}

        # subscribers
        self.feedback_subscribers = {}
        info = "Initialized subscribers:"
        for joint, topic in self.feedback_topics.items():
            self.feedback_subscribers[joint] = rospy.Subscriber(
                topic,
                SiriusJointState,
                partial(self._process_hardware_state, joint_name=joint),
            )
            info = info + f"\n    - {joint}: {topic}"

        rospy.loginfo(info)

        # publishers
        self.command_publishers = {}
        info = "Initialized publishers:"
        for joint, topic in self.command_topics.items():
            self.command_publishers[joint] = rospy.Publisher(
                topic, SiriusJointState, queue_size=10
            )
            info = info + f"\n    - {joint}: {topic}"

        rospy.loginfo(info)

    def _pre_run(self):
        duration = rospy.Duration(1 / self._rate)
        callback = lambda x: self._publish_hardware_state()
        rospy.Timer(duration, callback)

    def _process_hardware_state(self, state: SiriusJointState, joint_name=""):
        self.joint_positions[joint_name] = state.position[0]

    def _publish_hardware_state(self):
        msg = JointState()

        msg.header.stamp = rospy.Time.now()
        msg.name = []
        msg.position = []
        for joint, position in self.joint_positions.items():
            msg.name.append(joint)
            msg.position.append(position)

        self._publish_state(msg)

    def _send_hardware_command(self, joint_state: JointState):
        for index, name in enumerate(joint_state.name):
            if name not in self.command_topics.keys():
                rospy.logwarn(
                    f"Recieved joint_state command for unsupported joint: {name}"
                )
                continue

            if (
                not hasattr(self, "command_publishers")
                or name not in self.command_publishers.keys()
            ):
                continue

            msg = SiriusJointState()
            msg.header.stamp = rospy.Time.now()
            msg.position = []
            msg.velocity = []
            msg.effort = []
            if index < len(joint_state.position):
                msg.position.append(joint_state.position[index])
                if name == "arm3_tool":
                    continue
            if index < len(joint_state.velocity):
                msg.velocity.append(joint_state.velocity[index])
            if index < len(joint_state.effort):
                # we interpret input effort as duty, even though it's technically incorrect
                msg.duty.append(joint_state.effort[index])


            self.command_publishers[name].publish(msg)


if __name__ == "__main__":
    try:
        ManipRoboclawInterface().run()
    except rospy.ROSInterruptException:
        pass
