#!/usr/bin/env python3

import rospy
from random import uniform
from geometry_msgs.msg import Twist
import sys
import select


class Node:

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=True)
        rospy.loginfo("Press Enter to reverse direction\n\n\033[F\033[F")
        self.rate = rospy.Rate(20)
        self.publisher = rospy.Publisher(f"cmd_vel", Twist, queue_size=10)
        self.msg = Twist()

        self.max_linear_velocity = rospy.get_param(f"~max_linear_velocity", 1)
        self.max_angular_velocity = rospy.get_param(f"~max_angular_velocity",
                                                    1)
        self.max_period = rospy.get_param(f"~max_period", 10)
        rospy.on_shutdown(self.shutdown)

        self.deadline = 0
        self.step()

    def run(self):
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                sys.stdin.readline()
                self.msg.linear.x = -self.msg.linear.x
                self.msg.angular.z = -self.msg.angular.z
                print('\033[K\033[F', end='\r')
            self.publisher.publish(self.msg)
            rospy.loginfo(
                "Linear: " + f"{self.msg.linear.x:.2f}".rjust(5) +
                f" Angular: " + f"{self.msg.angular.z:.2f}".rjust(5) +
                f"  Period: {self.deadline - rospy.get_time():.2f}\033[F")
            self.rate.sleep()

    def step(self, event=None):
        if not rospy.is_shutdown():
            self.msg.linear.x = uniform(-self.max_linear_velocity,
                                        self.max_linear_velocity)
            self.msg.angular.z = uniform(-self.max_angular_velocity,
                                         self.max_angular_velocity)
            period = uniform(0, self.max_period)
            self.deadline = period + rospy.get_time()
            rospy.loginfo(
                "Linear: " + f"{self.msg.linear.x:.2f}".rjust(5) +
                f" Angular: " + f"{self.msg.angular.z:.2f}".rjust(5) +
                f"  Period: {self.deadline - rospy.get_time():.2f}\033[F")
            rospy.Timer(rospy.Duration(period), self.step, True)

    def shutdown(self):
        pass


if __name__ == '__main__':
    Node("random_traversal").run()
    print("")
