#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState


DEFAULT_JOINT_STATE = JointState()
DEFAULT_JOINT_STATE.name = [f"joint{i}" for i in range(1, 7)]
# DEFAULT_JOINT_STATE.name = [f"joint{i}" for i in range(1, 4)]
# DEFAULT_JOINT_STATE.name = [f"joint{i}" for i in range(4, 7)]
DEFAULT_JOINT_STATE.position = [0.0] * 6


class MockManipulatorNode:
    """
    A mock ROS 1 manipulator node that republishes received joint states.

    Subscribes to 'set_joint_states' (JointState) and publishes the
    most recently received message to 'joint_states' (JointState)
    at a fixed rate.
    """
    def __init__(self):
        node_name = "mock_manip_node"
        rospy.init_node(node_name)

        rospy.loginfo(f"Starting {node_name}...")

        self.publish_rate = rospy.get_param('~publish_rate', 50.0) # Hz
        set_topic_name = rospy.get_param('~set_joint_states_topic', '/manip/set_joint_states')
        state_topic_name = rospy.get_param('~joint_states_topic', '/manip/joint_states')

        self._latest_joint_state = DEFAULT_JOINT_STATE

        self._state_publisher = rospy.Publisher(state_topic_name, JointState, queue_size=10)
        rospy.loginfo(f"Advertising topic {rospy.resolve_name(state_topic_name)} (JointState)")

        self._set_state_subscriber = rospy.Subscriber(set_topic_name, JointState, self._set_joint_states_callback)
        rospy.loginfo(f"Subscribing to topic {rospy.resolve_name(set_topic_name)} (JointState)")

        self._rate = rospy.Rate(self.publish_rate)
        rospy.loginfo(f"Publishing robot state at {self.publish_rate} Hz")


    def _set_joint_states_callback(self, msg: JointState):
        """
        Callback for the set_joint_states topic.
        Stores the most recently received JointState message.
        """
        self._latest_joint_state = msg

    def run(self):
        """
        Main loop for the node. Publishes the latest joint state at the configured rate.
        """
        while not rospy.is_shutdown():
            # Get the latest joint state thread-safely
            latest_msg = self._latest_joint_state

            # If we have received a message, publish it
            if latest_msg is not None:
                # It's good practice to update the timestamp for published messages
                latest_msg.header.stamp = rospy.Time.now()
                self._state_publisher.publish(latest_msg)
                # rospy.loginfo("Published joint state") # Uncomment for verbose output

            # Sleep to maintain the publishing rate
            try:
                self._rate.sleep()
            except rospy.ROSInterruptException:
                # This exception is thrown when rospy.signal_shutdown() is called
                break # Exit the loop

        rospy.loginfo("MockManipulatorNode shutting down.")


if __name__ == '__main__':
    try:
        node = MockManipulatorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()