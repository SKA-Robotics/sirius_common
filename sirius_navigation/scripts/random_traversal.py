#!/usr/bin/env python3

import rospy
from random import uniform
from geometry_msgs.msg import Twist

class Node:

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=True)

        self.rate = rospy.Rate(20)
        self.publisher = rospy.Publisher(f"cmd_vel", Twist, queue_size=10)
        self.msg = Twist() 

        self.max_linear_velocity = rospy.get_param(f"~max_linear_velocity", 1)
        self.max_angular_velocity = rospy.get_param(f"~max_angular_velocity", 1)
        self.max_period = rospy.get_param(f"~max_period", 10)
        rospy.on_shutdown(self.shutdown)
        self.step()

    def run(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.msg)
            self.rate.sleep()

    def step(self, event=None):
        if not rospy.is_shutdown():
            self.msg.linear.x  = uniform(-self.max_linear_velocity, self.max_linear_velocity)
            self.msg.angular.z = uniform(-self.max_angular_velocity, self.max_angular_velocity)
            period = uniform(0, self.max_period)
            rospy.Timer(rospy.Duration(period), self.step, True) 

    def shutdown(self):
        pass


if __name__ == '__main__':
    Node("random_traversal").run()
