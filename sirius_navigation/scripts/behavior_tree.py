#!/usr/bin/env python3

import threading

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty


class NodeStatus:
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class Sequence:
    def __init__(self, children):
        self.children = children

    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.SUCCESS:
                return status
        return NodeStatus.SUCCESS


class Fallback:
    def __init__(self, children):
        self.children = children

    def tick(self):
        for child in self.children:
            status = child.tick()
            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.FAILURE


class GoalNode:
    def __init__(self, behavior_tree):
        self.behavior_tree = behavior_tree

    def tick(self):
        if self.behavior_tree.last_goal is None:
            return NodeStatus.SUCCESS

        if self.behavior_tree.goal_running:
            return NodeStatus.RUNNING

        self.behavior_tree.send_goal(self.behavior_tree.last_goal)
        return NodeStatus.RUNNING


class ReplanningFailureGuard:
    def __init__(self, behavior_tree):
        self.behavior_tree = behavior_tree

    def tick(self):
        if self.behavior_tree.replanning_failures >= self.behavior_tree.max_failed_replans:
            return NodeStatus.FAILURE
        return NodeStatus.SUCCESS


class RecoveryNode:
    def __init__(self, behavior_tree):
        self.behavior_tree = behavior_tree

    def tick(self):
        if self.behavior_tree.recovery_in_progress:
            return NodeStatus.RUNNING

        if self.behavior_tree.last_goal is not None:
            self.behavior_tree.trigger_recovery()
            return NodeStatus.RUNNING

        return NodeStatus.SUCCESS


class SimpleBehaviorTree:
    def __init__(self):
        rospy.init_node("simple_behavior_tree")
        self.max_failed_replans = rospy.get_param("~max_failed_replans", 10)
        self.recovery_duration = rospy.get_param("~recovery_duration", 2.0)
        self.recovery_rotation_speed = rospy.get_param("~recovery_rotation_speed", 0.5)
        self.replanning_failures = 0
        self.goal_running = False
        self.recovery_in_progress = False
        self.last_goal = None

        self.lock = threading.Lock()
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.move_base_action = SimpleActionClient("move_base/move_base", MoveBaseAction)

        try:
            self.move_base_action.wait_for_server(rospy.Duration(10.0))
        except rospy.ROSException:
            rospy.logwarn("move_base/move_base server not available yet; waiting for it.")
            self.move_base_action.wait_for_server()

        rospy.loginfo("Behavior tree connected to move_base/move_base")
        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)

        self.root = Sequence([
            GoalNode(self),
            Fallback([
                ReplanningFailureGuard(self),
                RecoveryNode(self),
            ]),
        ])

    def goal_callback(self, msg):
        with self.lock:
            self.last_goal = msg
            self.replanning_failures = 0
            rospy.loginfo("Received new goal: x=%.2f y=%.2f", msg.pose.position.x, msg.pose.position.y)

    def send_goal(self, pose):
        if pose is None:
            return

        goal = MoveBaseGoal(target_pose=pose)
        self.goal_running = True
        rospy.loginfo("Sending goal to move_base/move_base")
        self.move_base_action.send_goal(goal, done_cb=self.goal_done)

    def goal_done(self, state, result):
        with self.lock:
            if state == GoalStatus.SUCCEEDED:
                self.goal_running = False
                self.replanning_failures = 0
                rospy.loginfo("Goal succeeded")
                return

            failed_states = {
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.PREEMPTED,
                GoalStatus.LOST,
            }

            if state in failed_states:
                self.goal_running = False
                self.replanning_failures += 1
                rospy.logwarn(
                    "Navigation failed (state=%d). replanning_failures=%d/%d",
                    state,
                    self.replanning_failures,
                    self.max_failed_replans,
                )

                if self.replanning_failures >= self.max_failed_replans:
                    self.trigger_recovery()
                elif self.last_goal is not None:
                    self.send_goal(self.last_goal)
                return

            rospy.logwarn("Goal ended with state=%d; keeping current goal state.", state)
            self.goal_running = False

    def trigger_recovery(self):
        with self.lock:
            if self.recovery_in_progress:
                return
            self.recovery_in_progress = True

        rospy.logwarn(
            "Too many failed replanning attempts (%d). Starting recovery behavior.",
            self.replanning_failures,
        )

        self.move_base_action.cancel_all_goals()
        self.goal_running = False

        self.rotate_in_place(self.recovery_rotation_speed, self.recovery_duration)

        try:
            rospy.wait_for_service("move_base/clear_costmaps", timeout=2.0)
            clear_costmaps = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
            clear_costmaps()
            rospy.loginfo("Costmaps cleared")
        except (rospy.ROSException, rospy.ServiceException):
            rospy.logwarn("move_base/clear_costmaps service not available; continuing without it")

        self.replanning_failures = 0

        if self.last_goal is not None:
            rospy.sleep(1.0)
            self.send_goal(self.last_goal)

        self.recovery_in_progress = False

    def rotate_in_place(self, angular_speed, duration):
        command = Twist()
        command.angular.z = angular_speed
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)

        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(command)
            rate.sleep()

        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.root.tick()
            rate.sleep()


if __name__ == "__main__":
    try:
        tree = SimpleBehaviorTree()
        tree.spin()
    except rospy.ROSInterruptException:
        pass
