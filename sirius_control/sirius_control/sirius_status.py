import rclpy
from rclpy.node import Node
from industrial_msgs.msg import RobotStatus, TriState, RobotMode
from std_msgs.msg import String


class SiriusStatus(Node):

    def __init__(self):
        super().__init__('sirius_status')

        self.initialize_robot_status()
        
        self.joy_subscription_ = self.create_subscription(
        String,  'joy_multiplexer/selected_output', self.joy_multiplexer_callback, 10)

        self.subscription_ = self.create_subscription(
        String, 'relaxing_middleware/state', self.relaxing_middleware_callback, 10)

        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        
    def initialize_robot_status(self):
        self.robot_status = RobotStatus()
        self.robot_status.e_stopped.val = TriState.FALSE
        self.robot_status.drives_powered.val = TriState.TRUE
        self.robot_status.motion_possible.val= TriState.TRUE
        self.robot_status.in_motion.val = TriState.FALSE
        self.robot_status.in_error.val = TriState.FALSE
        self.robot_status.mode.val = RobotMode.UNKNOWN
        self.robot_status.error_codes = []


    def joy_multiplexer_callback(self, msg):
        if msg.data == "__none":
            self.robot_status.mode.val = RobotMode.UNKNOWN

        elif msg.data == "joy_diff_drive":
            self.robot_status.mode.val = RobotMode.MANUAL


        self.publish_sirius_status()

    def relaxing_middleware_callback(self, msg):
        if msg.data == "Locked":
            self.robot_status.in_motion.val = TriState.FALSE
            self.robot_status.drives_powered.val = TriState.TRUE

        elif msg.data == "Idle":
            self.robot_status.drives_powered.val= TriState.FALSE
            self.robot_status.in_motion.val = TriState.FALSE

        elif msg.data == "Running" or msg.data == "Breaking" or msg.data == "Relaxing":
            self.robot_status.in_motion.val = TriState.TRUE
            self.robot_status.drives_powered.val = TriState.TRUE

        self.publish_sirius_status()
        

    def publish_sirius_status(self):
        self.robot_status.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.robot_status)


def main(args=None):
    rclpy.init(args=args)
    sirius_status = SiriusStatus()
    try:
        rclpy.spin(sirius_status)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()
    sirius_status.destroy_node()

if __name__ == '__main__':
    main()