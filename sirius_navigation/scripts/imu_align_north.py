#!/usr/bin/env python3

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import PyKDL
import rospy
from math import pi


class ImuAlignNorth:

    def __init__(self):
        rospy.init_node('imu_align_north', anonymous=True)

        self.declination = rospy.get_param('~declination', 0.0) * pi / 180.0
        self.imu_sub = rospy.Subscriber('imu/data', Imu, self.imu_callback)
        self.imu_pub = rospy.Publisher('imu_aligned/data', Imu, queue_size=10)

    def imu_callback(self, data):
        orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                data.orientation.y,
                                                data.orientation.z,
                                                data.orientation.w)

        transform = PyKDL.Rotation.RPY(0, 0, self.declination)

        aligned_orientation = transform * orientation

        x, y, z, w = aligned_orientation.GetQuaternion()

        self.imu_pub.publish(
            Imu(header=data.header,
                orientation=Quaternion(x, y, z, w),
                orientation_covariance=data.orientation_covariance,
                angular_velocity=data.angular_velocity,
                angular_velocity_covariance=data.angular_velocity_covariance,
                linear_acceleration=data.linear_acceleration,
                linear_acceleration_covariance=data.
                linear_acceleration_covariance))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    ImuAlignNorth().run()
