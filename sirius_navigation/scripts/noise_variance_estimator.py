#!/usr/bin/env python3
"""Module for calculating the noise variance matrix
of the localization topics"""

from collections import OrderedDict
from typing import Final
import sys
import tf2_geometry_msgs
import tf2_ros
import rosbag
import rospy
import PyKDL
import bisect
from tf.transformations import euler_from_quaternion
from math import pi

from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    Pose,
    Twist,
    Vector3,
    Quaternion)

from nav_msgs.msg import Odometry
from textwrap import indent


# Constants
LOCALIZATION_TYPES: Final = {'geometry_msgs/PoseWithCovarianceStamped',
                             'geometry_msgs/TwistWithCovarianceStamped',
                             'nav_msgs/Odometry', 'sensor_msgs/Imu'}


class NoiseEstimatorError(Exception):
    """Exception raised for errors in the NoiseEstimator module."""

    def __init__(self, message, hint=None):
        super().__init__(message)
        self.hint = hint

    def log(self):
        """Log the exception with ros"""
        rospy.logerr(self)
        if self.hint is not None:
            rospy.loginfo(self.hint)


class StampedMessages:
    def __init__(self, bag, topic):
        messages = OrderedDict()

        for _, message, timestamp in bag.read_messages(topics=[topic]):
            messages[timestamp] = message

        messages = OrderedDict(sorted(messages.items()))
        self.timestamps = list(messages.keys())
        self.messages = list(messages.values())

    def retrieve(self, timestamp):
        # Perform binary search to find the closest earlier timestamp
        index = bisect.bisect(self.timestamps, timestamp)
        if index:
            # If there is an earlier timestamp, return the associated value
            return self.messages[index - 1]
        else:
            # If there is no earlier timestamp, return None
            return None

    def __getitem__(self, timestamp):
        return self.retrieve(timestamp)

    def __len__(self):
        return len(self.messages)


def angle_diff(theta1, theta2):
    d = theta2 - theta1
    d = (d + pi) % (2 * pi) - pi
    return d


def get_localization_topics(bag, ground_truth_topic):
    localization_topics = {topic: value.msg_type for topic, value
                           in bag.get_type_and_topic_info().topics.items()
                           if value.msg_type in LOCALIZATION_TYPES
                           and topic != ground_truth_topic}
    if len(localization_topics) == 0:
        raise NoiseEstimatorError(
            "No localization topics found in bag",
            f"Available topics: {bag.get_type_and_topic_info().topics.keys()}")
    return localization_topics


def imuError(imu, ground_truth, tf_buffer):
    """Compare the imu message to the ground truth message"""
    ground_truth = transform_odometry_child_frame(
        ground_truth, imu.header.frame_id, tf_buffer)

    (imu_roll, imu_pitch, imu_yaw) = euler_from_quaternion(
        [imu.orientation.x, imu.orientation.y,
         imu.orientation.z, imu.orientation.w])
    (gt_roll, gt_pitch, gt_yaw) = euler_from_quaternion(
        [ground_truth.pose.pose.orientation.x,
         ground_truth.pose.pose.orientation.y,
         ground_truth.pose.pose.orientation.z,
         ground_truth.pose.pose.orientation.w])

    return [angle_diff(imu_roll, gt_roll),
            angle_diff(imu_pitch, gt_pitch),
            angle_diff(imu_yaw, gt_yaw),
            imu.angular_velocity.x - ground_truth.twist.twist.angular.x,
            imu.angular_velocity.y - ground_truth.twist.twist.angular.y,
            imu.angular_velocity.z - ground_truth.twist.twist.angular.z]


def initialize_bag(bag_path, ground_truth_topic):
    """Initialize and validate the bag file"""

    try:
        bag = rosbag.Bag(bag_path, 'r')
    except FileNotFoundError:
        raise NoiseEstimatorError("No such bag file",
                                  f"Bag file path: {bag_path}")
    except PermissionError:
        raise NoiseEstimatorError("No permission to read bag file",
                                  f"Bag file path: {bag_path}")
    except IsADirectoryError:
        raise NoiseEstimatorError("Bag file path is a directory not \
                                             a file",
                                  f"Bag file path: {bag_path}")
    except rosbag.bag.ROSBagException as error:
        raise NoiseEstimatorError(error,
                                  f"Bag file path: {bag_path}")

    bag_topics = bag.get_type_and_topic_info().topics
    if ground_truth_topic not in bag_topics.keys():
        raise NoiseEstimatorError(
            f"Ground truth topic\"{ground_truth_topic}\" not found in bag",
            f"Available topics: {bag_topics.keys()}")

    if bag_topics[ground_truth_topic].msg_type != "nav_msgs/Odometry":
        raise NoiseEstimatorError(
            f"Ground truth topic\"{ground_truth_topic} \
            \" is not of type nav_msgs/Odometry",
            f"Ground truth topic type: \
            {bag_topics[ground_truth_topic].msg_type}")

    return bag


def noiseVariance(bag, topic, calculate_error, ground_truth, tf_buffer):
    # Initialize error_squared
    ground_truth_message = None
    bag_iterator = bag.read_messages(topics=[topic])
    while ground_truth_message is None:
        _, message, stamp = next(bag_iterator)
        ground_truth_message = ground_truth[stamp]
    error_squared = [0] * len(calculate_error(message, ground_truth_message,
                                              tf_buffer))
    samples = 0
    for _, message, stamp in bag.read_messages(topics=[topic]):
        ground_truth_message = ground_truth[stamp]
        if ground_truth_message is not None:
            error = calculate_error(message, ground_truth_message, tf_buffer)
            error_squared = [error_squared[i] +
                             error[i]**2 for i in range(len(error))]
            samples += 1
    return [error_squared[i] /
            samples for i in range(len(error_squared))]


def odometryError(odom, ground_truth, tf_buffer):
    """Compare the odometry message to the ground truth message"""
    if odom.child_frame_id != "":
        twist_ground_truth = transform_odometry_child_frame(
            ground_truth, odom.child_frame_id, tf_buffer)
    else:
        twist_ground_truth = ground_truth
    return [*poseError(odom.pose, ground_truth),
            *twistError(odom.twist, twist_ground_truth)]


def poseError(pose, ground_truth):
    # pose can be stamped and with covariance, extract the pose
    while hasattr(pose, 'pose'):
        pose = pose.pose

    (pose_roll, pose_pitch, pose_yaw) = euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w])
    (gt_roll, gt_pitch, gt_yaw) = euler_from_quaternion(
        [ground_truth.pose.pose.orientation.x,
         ground_truth.pose.pose.orientation.y,
         ground_truth.pose.pose.orientation.z,
         ground_truth.pose.pose.orientation.w])

    return [pose.position.x - ground_truth.pose.pose.position.x,
            pose.position.y - ground_truth.pose.pose.position.y,
            pose.position.z - ground_truth.pose.pose.position.z,
            angle_diff(pose_roll, gt_roll),
            angle_diff(pose_pitch, gt_pitch),
            angle_diff(pose_yaw, gt_yaw)]


def prepare_tf_buffer(bag):
    tf_buffer = tf2_ros.Buffer()
    for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
        for transform in msg.transforms:
            # Somehow the transform is not geometry_msgs/TransformStamped
            # and tf will show warnings if types do not match
            transform.transform.translation = Vector3(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z)
            transform.transform.rotation = Quaternion(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w)
            if topic == '/tf_static':
                tf_buffer.set_transform_static(transform, "")
            else:
                tf_buffer.set_transform(transform, "")
    return tf_buffer


def reprImuNoise(variance):
    return ("roll:".ljust(20) + f"{variance[0]:.4E}\n" +
            "pitch:".ljust(20) + f"{variance[1]:.4E}\n" +
            "yaw:".ljust(20) + f"{variance[2]:.4E}\n" +
            "roll velocity:".ljust(20) + f"{variance[3]:.4E}\n" +
            "pitch velocity:".ljust(20) + f"{variance[4]:.4E}\n" +
            "yaw velocity:".ljust(20) + f"{variance[5]:.4E}")


def reprOdometryNoise(variance):
    return f"{reprPoseNoise(variance[:6])} \n" \
        f"{reprTwistNoise(variance[6:])}"


def reprPoseNoise(variance):
    return ("x:".ljust(20) + f"{variance[0]:.4E}\n" +
            "y:".ljust(20) + f"{variance[1]:.4E}\n" +
            "z:".ljust(20) + f"{variance[2]:.4E}\n" +
            "roll:".ljust(20) + f"{variance[3]:.4E}\n" +
            "pitch:".ljust(20) + f"{variance[4]:.4E}\n" +
            "yaw:".ljust(20) + f"{variance[5]:.4E}")


def reprTwistNoise(variance):
    return ("x velocity:".ljust(20) + f"{variance[0]:.4E}\n" +
            "y velocity:".ljust(20) + f"{variance[1]:.4E}\n" +
            "z velocity:".ljust(20) + f"{variance[2]:.4E}\n" +
            "roll velocity:".ljust(20) + f"{variance[3]:.4E}\n" +
            "pitch velocity:".ljust(20) + f"{variance[4]:.4E}\n" +
            "yaw velocity:".ljust(20) + f"{variance[5]:.4E}")


def transform_odometry_child_frame(msg, target_frame, tf_buffer):
    """Transform the child frame of the odom message to the target frame."""

    target_pose = transform_pose_child_frame(
        msg.pose.pose, target_frame, msg.child_frame_id,
        tf_buffer, msg.header.stamp)
    target_twist = transform_twist_child_frame(
        msg.twist.twist, target_frame, msg.child_frame_id,
        tf_buffer, msg.header.stamp)

    return Odometry(header=msg.header, child_frame_id=target_frame,
                    pose=PoseWithCovariance(pose=target_pose),
                    twist=TwistWithCovariance(twist=target_twist))


def transform_pose_child_frame(pose, target_frame, child_frame,
                               tf_buffer, time):
    """Transform the child frame of the odom message to the target frame."""

    # Convert the pose to a PyKDL Frame
    child_to_parent_transform = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w),
        PyKDL.Vector(
            pose.position.x,
            pose.position.y,
            pose.position.z))

    # Lookup the transform from the target frame to the child frame
    # and convert it to a PyKDL Frame
    # This is the same as the target pose in the child frame
    target_to_child_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(child_frame,
                                   target_frame, time))

    # Combine the two transforms to get the target pose in the parent frame
    target_to_parent_transform = child_to_parent_transform * \
        target_to_child_transform

    # Convert the transform to a geometry_msgs/Pose
    target_pose = Pose()
    target_pose.position.x = target_to_parent_transform[(0, 3)]
    target_pose.position.y = target_to_parent_transform[(1, 3)]
    target_pose.position.z = target_to_parent_transform[(2, 3)]
    (target_pose.orientation.x, target_pose.orientation.y,
     target_pose.orientation.z, target_pose.orientation.w) = \
        target_to_parent_transform.M.GetQuaternion()

    return target_pose


def transform_twist(twist, transform):
    linear_velocity = PyKDL.Vector(
        twist.linear.x, twist.linear.y, twist.linear.z)
    angular_velocity = PyKDL.Vector(
        twist.angular.x, twist.angular.y, twist.angular.z)

    translation = PyKDL.Vector(
        transform[0, 3], transform[1, 3], transform[2, 3])
    rotation = PyKDL.Rotation(transform[0, 0], transform[1, 0],
                              transform[2, 0], transform[0, 1],
                              transform[1, 1], transform[2, 1],
                              transform[0, 2], transform[1, 2],
                              transform[2, 2])

    target_linear_velocity = rotation * \
        linear_velocity + angular_velocity * translation
    target_angular_velocity = rotation * angular_velocity

    return Twist(
        Vector3(target_linear_velocity[0],
                target_linear_velocity[1],
                target_linear_velocity[2]),
        Vector3(target_angular_velocity[0],
                target_angular_velocity[1],
                target_angular_velocity[2]))


def transform_twist_child_frame(twist, target_frame, child_frame,
                                tf_buffer, time):

    # Lookup the transform from the child frame to the target frame
    # and convert it to a PyKDL Frame
    child_to_target_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(target_frame, child_frame, time))

    twist = transform_twist(twist, child_to_target_transform)

    return twist


def twistError(twist, ground_truth):
    # twist can be stamped and with covariance, extract the twist
    while hasattr(twist, 'twist'):
        twist = twist.twist

    return [twist.linear.x - ground_truth.twist.twist.linear.x,
            twist.linear.y - ground_truth.twist.twist.linear.y,
            twist.linear.z - ground_truth.twist.twist.linear.z,
            twist.angular.x - ground_truth.twist.twist.angular.x,
            twist.angular.y - ground_truth.twist.twist.angular.y,
            twist.angular.z - ground_truth.twist.twist.angular.z]


localization_error = {'nav_msgs/Odometry': odometryError,
                      'sensor_msgs/Imu': imuError,
                      'geometry_msgs/PoseStamped': poseError,
                      'geometry_msgs/TwistStamped': twistError}
repr_localization_noise = {'nav_msgs/Odometry': reprOdometryNoise,
                           'sensor_msgs/Imu': reprImuNoise,
                           'geometry_msgs/PoseStamped': reprPoseNoise,
                           'geometry_msgs/TwistStamped': reprTwistNoise}

if __name__ == '__main__':
    rospy.init_node("noise_variance_estimator")
    # Parameters
    ground_truth_topic = rospy.get_param("~ground_truth", '/ground_truth')

    try:
        bag_path = rospy.get_param("~bag_path")
    except KeyError:
        rospy.logfatal("No bag file path provided")
        sys.exit()

    try:
        bag = initialize_bag(bag_path, ground_truth_topic)
        localization_topics = get_localization_topics(bag, ground_truth_topic)
    except NoiseEstimatorError as error:
        error.log()
        sys.exit()

    tf_buffer = prepare_tf_buffer(bag)

    max_topic_length = max([len(topic) for topic in localization_topics])
    rospy.loginfo("\nLocalization topics found:" + ''.join(
        ["\n\t" + topic.ljust(max_topic_length + 5) + message_type
         for topic, message_type
         in localization_topics.items()]))

    ground_truth = StampedMessages(bag, ground_truth_topic)

    for topic, message_type in localization_topics.items():
        variance = noiseVariance(bag, topic,
                                 localization_error[message_type],
                                 ground_truth, tf_buffer)
        rospy.loginfo(
            f"\nNoise variance for {topic}: \n" +
            indent(repr_localization_noise[message_type](variance), "\t"))
