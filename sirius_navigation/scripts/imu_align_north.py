#!/usr/bin/env python3

from math import pi
import PyKDL

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class ImuAlignNorth(Node):

    def __init__(self):
        super().__init__('imu_align_north')

        self.declare_parameter('declination', 0.0)
        declination_deg = self.get_parameter('declination').value
        self._declination = declination_deg * pi / 180.0

        self._imu_sub = self.create_subscription(
            Imu, 'imu/data', self._imu_callback, 10)
        self._imu_pub = self.create_publisher(Imu, 'imu_aligned/data', 10)

    def _imu_callback(self, data: Imu):
        orientation = PyKDL.Rotation.Quaternion(
            data.orientation.x, data.orientation.y,
            data.orientation.z, data.orientation.w)
        transform = PyKDL.Rotation.RPY(0, 0, self._declination)
        aligned = transform * orientation
        x, y, z, w = aligned.GetQuaternion()

        msg = Imu()
        msg.header = data.header
        msg.orientation = Quaternion(x=x, y=y, z=z, w=w)
        msg.orientation_covariance = data.orientation_covariance
        msg.angular_velocity = data.angular_velocity
        msg.angular_velocity_covariance = data.angular_velocity_covariance
        msg.linear_acceleration = data.linear_acceleration
        msg.linear_acceleration_covariance = data.linear_acceleration_covariance
        self._imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAlignNorth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
