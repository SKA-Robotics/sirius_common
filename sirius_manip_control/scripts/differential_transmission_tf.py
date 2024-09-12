#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
class DifferentialTransmissionStatePublisher():
        def __init__(self) -> None:
                self.first_input_joint = rospy.get_param('~first_input_joint')
                self.second_input_joint = rospy.get_param('~second_input_joint')
                self.first_output_joint = rospy.get_param('~first_output_joint')
                self.second_output_joint = rospy.get_param('~second_output_joint')
                self.first_output_offset = rospy.get_param('~first_output_offset')
                self.second_output_offset = rospy.get_param('~second_output_offset')
                self.joint_state_subscriber = rospy.Subscriber('/input_joint_states',JointState,self.joint_state_callback)
                self.joint_state_publisher = rospy.Publisher('/output_joint_states', JointState, queue_size=10)



        def joint_state_callback(self, msg):
                first_input_state = None
                second_input_state = None
                joint_states = []
                for i in range(len(msg.name)):
                        if msg.name[i] == self.first_input_joint:
                                first_input_state = {'name': msg.name[i], 'position': msg.position[i]}
                        elif msg.name[i] == self.second_input_joint:
                                second_input_state = {'name': msg.name[i], 'position': msg.position[i]}
                        else:
                                joint_states.append({'name': msg.name[i], 'position': msg.position[i]})
                if first_input_state is not None and second_input_state is not None:
                        first_output_state = {'name': self.first_output_joint, 'position': (first_input_state['position'] + second_input_state['position'])/2 + self.first_output_offset}
                        second_output_state = {'name': self.second_output_joint, 'position': (first_input_state['position'] - second_input_state['position'])/2  + self.second_output_offset}
                        joint_states.append(first_output_state)
                        joint_states.append(second_output_state)
                        self.publish_joint_states(msg.header, joint_states)
        def publish_joint_states(self, header, joint_states):
                msg = JointState(header, [joint['name'] for joint in joint_states],[joint['position'] for joint in joint_states],[],[])
                self.joint_state_publisher.publish(msg)
        def run(self):
                rospy.spin()

if __name__ == '__main__':
        rospy.init_node('differential_transmission_state_publisher')
        DifferentialTransmissionStatePublisher().run()

