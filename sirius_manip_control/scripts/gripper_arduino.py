#!/usr/bin/python3
import serial
import rospy
import traceback
from sensor_msgs.msg import Joy


class GripperController:

    def __init__(self, portname, baudrate):
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.angle = 0.5
        self.speed = 0.03
        try:
            self.serial = serial.Serial(portname, baudrate)
        except serial.SerialException as e:
            rospy.logerr("Could not open serial port %s: %s" % (portname, traceback.format_exc()))
            exit(1)

    def run(self):
        while not rospy.is_shutdown():
            if (self.serial.in_waiting):
                print(self.serial.readline())

    def joy_callback(self, msg):
        self.angle = clamp(self.angle + msg.axes[-1] * self.speed, -1, 1)
        rospy.loginfo(self.angle)
        command = int((self.angle + 1.0) * 123) + 1
        rospy.loginfo(command)
        self.serial.write([128, clamp(command, 1, 254), 192])

    def __del__(self):
        self.serial.close()


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


if __name__ == '__main__':
    try:
        rospy.init_node('gripper')
        driver = GripperController(rospy.get_param("~port_name", "/dev/ttyACM0"), rospy.get_param("~baudrate", 9600))
        driver.run()
    except rospy.ROSInterruptException:
        pass
