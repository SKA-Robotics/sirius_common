import rclpy
from rclpy.node import Node
from industrial_msgs.msg import RobotStatus, TriState, RobotMode
from std_msgs.msg import String


class SiriusStatus(Node):

    def __init__(self):
        super().__init__('sirius_status')
        self.send_status = self.declare_parameter('send_status', '/sirius_status').value
        self.default_receive_topics = [
            '/joy_multiplexer/selected_output',
            '/relaxing_middleware/state'
        ]
        self.receive_topics = self.declare_parameter('receive_topics', self.default_receive_topics).value
        self.initialize_robot_status()
        self.create_subscriptions()
        self.publisher = self.create_publisher(RobotStatus, self.send_status, 10)
        
    def initialize_robot_status(self):
        self.robot_status = RobotStatus()
        self.robot_status.e_stopped.val = TriState.FALSE
        self.robot_status.drives_powered.val = TriState.TRUE
        self.robot_status.motion_possible.val= TriState.TRUE
        self.robot_status.in_motion.val = TriState.FALSE
        self.robot_status.in_error.val = TriState.FALSE
        self.robot_status.mode.val = RobotMode.UNKNOWN
        self.robot_status.error_codes = []


    def topic_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)
        if msg.data == "Locked":
            self.robot_status.in_motion.val = TriState.FALSE
            self.robot_status.drives_powered.val = TriState.TRUE

        elif msg.data == "__none":
            self.robot_status.mode.val = RobotMode.UNKNOWN

        elif msg.data == "joy_diff_drive":
            self.robot_status.mode.val = RobotMode.MANUAL

        elif msg.data == "autonomic":
            self.robot_status.mode.val = RobotMode.AUTO

        elif msg.data == "Idle":
            self.robot_status.drives_powered.val= TriState.FALSE
            self.robot_status.in_motion.val = TriState.FALSE

        elif msg.data == "Running" or msg.data == "Breaking" or msg.data == "Relaxing":
            self.robot_status.in_motion.val = TriState.TRUE
            self.robot_status.drives_powered.val = TriState.TRUE

        self.publish_sirius_status()


    def create_subscriptions(self):
        for topic in self.receive_topics:
            self.subscription_ = self.create_subscription(
            String, topic, self.topic_callback, 10)


    def publish_sirius_status(self):
        self.publisher.publish(self.robot_status)
        self.get_logger().info('Published RobotStatus message')


def main(args=None):
    rclpy.init(args=args)
    sirius_status = SiriusStatus()
    rclpy.spin(sirius_status)
    sirius_status.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()