#!/usr/bin/env python3

import rospy

# Actions
from actionlib import SimpleActionClient

# Messages
from actionlib_msgs.msg import GoalStatus
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class SimpleGoalRelay:

    def __init__(self, name="simple_goal_relay"):
        rospy.init_node(name)

        rospy.logdebug("Connecting to action server move_base/move_base")
        self.move_base_action = SimpleActionClient("move_base/move_base", MoveBaseAction)
        self.move_base_action.wait_for_server()

        rospy.logdebug("Subscribing to topic move_base_simple/goal")
        self.goal_subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        in_progress_status = {GoalStatus.PENDING, GoalStatus.RECALLING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self.move_base_action.get_state() in in_progress_status:
            rospy.logdebug("Canceling previous move_base_action")
            self.move_base_action.cancel_all_goals()

        rospy.logdebug("Relaying simple goal message to move_base action")
        self.move_base_action.send_goal(MoveBaseGoal(target_pose=msg), self.move_base_done)

    def move_base_done(self, status, result):
        rospy.logdebug("Move base action completed with result [%d]: %s", result.outcome, result.message)

    def __del__(self):
        self.move_base_action.cancel_all_goals()


if __name__ == '__main__':
    node = SimpleGoalRelay()
    rospy.spin()