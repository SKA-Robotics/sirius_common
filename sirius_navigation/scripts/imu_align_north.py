#!/usr/bin/env python3

from sensor_msgs.msg import Imu
import PyKDL
import rospy


class ImuAlignNorth:

    def __init__(self):
        rospy.init_node('imu_align_north', anonymous=True)

        self.declination = rospy.get_param('~declination', 0.0)
        self.imu_sub = rospy.Subscriber('imu/data', Imu, self.imu_callback)
        self.imu_pub = rospy.Publisher('imu_aligned/data', Imu, queue_size=10)

    def imu_callback(self, data):
        orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                data.orientation.y,
                                                data.orientation.z,
                                                data.orientation.w)

        transform = PyKDL.Rotation.RPY(0, 0, self.declination)

        aligned_orientation = transform * orientation

        data.orientation.x, data.orientation.y, \
        data.orientation.z, data.orientation.w = \
                aligned_orientation.GetQuaternion()

        self.imu_pub.publish(data)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    ImuAlignNorth().run()
