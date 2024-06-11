#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from sirius_msgs.msg import JointState as SiriusJointState

class Node:
    def __init__(self):
        rospy.init_node("manip_jointstate_aggregator")
        self._joint_values = {}
        self._subscribers = []
        motors, joints, output_topic, self._rate = self._read_params()
        self._publisher = rospy.Publisher(output_topic, JointState, queue_size=10)
        for motor, joint in zip(motors, joints):
            self._add_subscriber(motor, joint)
    
    def run(self):
        self._start_cycle(self._rate)
        rospy.spin()
    
    def _read_params(self):
        motor_topics = rospy.get_param("~motor_topics")
        motors = motor_topics.values()
        joints = motor_topics.keys()
        output_topic = rospy.get_param("~jointstate_topic")
        rate = rospy.get_param("~publish_rate")
        return motors, joints, output_topic, rate
    
    def _start_cycle(self, rate):
        duration = rospy.Duration(1 / rate)
        callback = lambda x: self._publish_joints()
        rospy.Timer(duration, callback)
    
    def _add_subscriber(self, motor, joint):
        callback = self._make_callback(joint)
        self._subscribers.append(rospy.Subscriber(motor, SiriusJointState, callback))
    
    def _make_callback(self, joint_name):
        def set_joint(msg):
            joint_value = msg.position[0]
            self._joint_values[joint_name] = joint_value
        return set_joint
    
    def _publish_joints(self):
        msg = self._generate_message()
        self._publisher.publish(msg)
    
    def _generate_message(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = []
        msg.position = []
        for joint_name in self._joint_values.keys():
            msg.name.append(joint_name)
            msg.position.append(self._joint_values[joint_name])
        return msg


if __name__=="__main__":
    Node().run()