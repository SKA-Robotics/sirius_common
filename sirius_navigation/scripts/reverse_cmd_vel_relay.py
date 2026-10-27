#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class NegCmdVelRelay:
    def __init__(self):
        rospy.init_node('rev_cmd_vel_relay')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/rev_cmd_vel', Twist, self.callback)

        rospy.loginfo("neg_cmd_vel_relay: subskrybuje /rev_cmd_vel, publikuje /cmd_vel")

    def callback(self, msg):
        out = Twist()

        # Oś X zanegowana
        out.linear.x = -msg.linear.x

        # Y bez zmian
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z

        # Theta (obrót wokół Z) bez zmian
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        self.pub.publish(out)


if __name__ == '__main__':
    try:
        NegCmdVelRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass