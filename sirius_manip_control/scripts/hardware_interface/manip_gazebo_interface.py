#!/usr/bin/python3
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from hardware_interface.manip_hardware_interface import (
    ManipHardwareInterface,
)


class ManipGazeboInterface(ManipHardwareInterface):
    def __init__(self):
        super().__init__()

    def _initialize_hardware_connection(self):
        self.command_topics = rospy.get_param("~command_topics", {})
        self.feedback_topic = rospy.get_param(
            "~feedback_topic", "/manipulator/joint_states"
        )

        # subscribers
        self.feedback_subscriber = rospy.Subscriber(
            self.feedback_topic, JointState, self._publish_state
        )
        rospy.loginfo(f"Initialized subscriber: {self.feedback_topic}")

        # publishers
        self.command_publishers = {}
        info = "Initialized publishers:"
        for joint, topic in self.command_topics.items():
            self.command_publishers[joint] = rospy.Publisher(
                topic, Float64, queue_size=10
            )
            info = info + f"\n    - {joint}: {topic}"

        rospy.loginfo(info)

    def _send_hardware_command(self, joint_state: JointState):
        for index, position in enumerate(joint_state.position):
            name = joint_state.name[index]
            if name not in self.command_topics.keys():
                rospy.logwarn(
                    f"Recieved joint_state command for unsupported joint: {name}"
                )
                continue

            self.command_publishers[name].publish(Float64(position))


if __name__ == "__main__":
    try:
        ManipGazeboInterface().run()
    except rospy.ROSInterruptException:
        pass
