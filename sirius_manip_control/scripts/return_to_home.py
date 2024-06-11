#!/usr/bin/python3
import rospy
from sirius_msgs.msg import JointState

topics = {
    "base_cyl": "roboclaw_driver/128/motor1/set_joint_state",
    "cyl_arm1": "roboclaw_driver/129/motor1/set_joint_state",
    "arm1_arm2": "roboclaw_driver/129/motor2/set_joint_state",
    "arm2_arm3": "roboclaw_driver/130/motor1/set_joint_state",
    "arm3_tool": "roboclaw_driver/130/motor2/set_joint_state",
}

publishers = [rospy.Publisher(topic, JointState, queue_size=10) for topic in topics.values()]

zero_message = JointState()
zero_message.position = [0]

def start_periodic_call(func):
    rospy.Timer(rospy.Duration(0.1), func)

def publish_first(_):
    zero_message.header.stamp = rospy.Time.now()
    publishers[0].publish(zero_message)

def publish_the_rest(_):
    zero_message.header.stamp = rospy.Time.now()
    for publisher in publishers[0:]:
        publisher.publish(zero_message)

rospy.init_node("return_to_home")
rospy.loginfo("Return to home procedure started...")

try:
    rospy.loginfo("Feel free to terminate when satisfied")
    start_periodic_call(publish_first)
    rospy.Timer(
        rospy.Duration(8),
        lambda x: start_periodic_call(publish_the_rest),
        oneshot=True)
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Done. I'm back home")