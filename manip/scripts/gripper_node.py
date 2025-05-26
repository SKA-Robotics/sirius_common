#!/usr/bin/python3
import rospy
from can_msgs.msg import Frame
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

def float_to_bits(self, value):
    value_bits = struct.pack("f", value)
    return struct.unpack("I", value_bits)[0]

def bits_to_float(self, value):
    value_bits = struct.pack("I", value)
    return struct.unpack("f", value_bits)[0]

class GripperNode:
    def __init__(self) -> None:
        rospy.init_node("gripper_node")

        self.device_id = 0x2c
        self.command_arbitration_id = 0x00
        self.report_arbitration_id = 0x01

        self.status_publisher = rospy.Publisher("/manip/joint_states", JointState, queue_size=10)
        self.command_subscriber = rospy.Subscriber("/gripper_cmd", Float32, self.command_callback, queue_size=10)

        self.send_publisher = rospy.Publisher("/sent_canbus_messages", Frame, queue_size=10)
        self.receive_subscriber = rospy.Subscriber("/received_canbus_messages", Frame, self.receive_raw_frame, queue_size=10)

    def send_frame(self, command_id, data, frame: Frame = None):
        if frame is None:
            frame = Frame()
            frame.id = (self.device_id << 5) | command_id
            frame.data = [data[i] if i < len(data) else 0 for i in range(8)]
            frame.dlc = len(data)

        self.send_publisher.publish(frame)

    def receive_raw_frame(self, frame: Frame):
        device_id = frame.id >> 5
        if self.device_id == device_id:
            command_id = frame.id & 0b11111
            self.receive_frame(command_id, frame.data, frame)

    def receive_frame(self, command_id, data, frame: Frame):
        if command_id == self.report_arbitration_id:
            msg = JointState()
            msg.name = ["gripper"]
            data = int.from_bytes(data, "big")
            msg.position = [bits_to_float(data)]
            msg.header.stamp = rospy.Time.now()
            self.status_publisher.publish(msg)
    
    def command_callback(self, msg):
        data = msg.data
        data = float_to_bits(data)
        data = data.to_bytes(8, byteorder="big")
        self.send_frame(self.command_arbitration_id, data)


if __name__=="__main__":
    node = GripperNode()
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        pass
    