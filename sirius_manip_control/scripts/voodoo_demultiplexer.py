#!/usr/bin/python3
import rospy

from sensor_msgs.msg import JointState
from sirius_msgs.msg import JointState as SiriusJointState

voodoo_topic = "voodoo_node/joint_states"
sirius_topics = {
    "base_cyl": "/roboclaw_driver/128/motor1/set_joint_state",
    "cyl_arm1": "/roboclaw_driver/129/motor1/set_joint_state",
    "arm1_arm2": "/roboclaw_driver/129/motor2/set_joint_state",
    "arm2_arm3": "/roboclaw_driver/130/motor1/set_joint_state",
    "arm3_tool": "/roboclaw_driver/130dddddd/motor2/set_joint_state",
    "gripper": "gripper",
}


publishers = {
    joint: rospy.Publisher(sirius_topics[joint], SiriusJointState) for joint in sirius_topics
}

def callback(message):
    for i in range(len(message.name)):
        new_message = SiriusJointState()
        new_message.header.stamp = rospy.Time.now()
        new_message.position = [message.position[i]]
        publishers[message.name[i]].publish(new_message)


if __name__=="__main__":
    rospy.init_node('voodoo_demultiplexer', anonymous=True)
    rospy.Subscriber(voodoo_topic, JointState, callback)
    rospy.spin()
