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
import numpy as np
import scipy.stats as stats
import yaml

from geometry_msgs.msg import (PoseWithCovariance, TwistWithCovariance, Pose,
                               Twist, Vector3, Quaternion)

from nav_msgs.msg import Odometry
from textwrap import indent

# Constants
LOCALIZATION_TYPES: Final = {
    'geometry_msgs/PoseWithCovarianceStamped',
    'geometry_msgs/TwistWithCovarianceStamped',
    'nav_msgs/Odometry',
    'sensor_msgs/Imu',
}


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

        for _, message, _ in bag.read_messages(topics=[topic]):
            timestamp = message.header.stamp
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
    d = (d + np.pi) % (2 * np.pi) - np.pi
    return d


def get_localization_topics(bag, ground_truth_topic):
    localization_topics = {
        topic: value.msg_type
        for topic, value in bag.get_type_and_topic_info().topics.items()
        if value.msg_type in LOCALIZATION_TYPES and topic != ground_truth_topic
    }
    if len(localization_topics) == 0:
        raise NoiseEstimatorError(
            "No localization topics found in bag",
            f"Available topics: {bag.get_type_and_topic_info().topics.keys()}")
    return localization_topics


def imuVectors(imu, ground_truth, tf_buffer):
    """Compare the imu message to the ground truth message"""
    ground_truth = transform_odometry_child_frame(ground_truth,
                                                  imu.header.frame_id,
                                                  tf_buffer)

    (imu_roll, imu_pitch, imu_yaw) = euler_from_quaternion([
        imu.orientation.x, imu.orientation.y, imu.orientation.z,
        imu.orientation.w
    ])
    (gt_roll, gt_pitch, gt_yaw) = euler_from_quaternion([
        ground_truth.pose.pose.orientation.x,
        ground_truth.pose.pose.orientation.y,
        ground_truth.pose.pose.orientation.z,
        ground_truth.pose.pose.orientation.w
    ])

    return np.array([
        [imu_roll, gt_roll],
        [imu_pitch, gt_pitch],
        [imu_yaw, gt_yaw],
        [imu.angular_velocity.x, ground_truth.twist.twist.angular.x],
        [imu.angular_velocity.y, ground_truth.twist.twist.angular.y],
        [imu.angular_velocity.z, ground_truth.twist.twist.angular.z],
    ])


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
        raise NoiseEstimatorError(
            "Bag file path is a directory not \
                                             a file",
            f"Bag file path: {bag_path}")
    except rosbag.bag.ROSBagException as error:
        raise NoiseEstimatorError(error, f"Bag file path: {bag_path}")

    bag_topics = bag.get_type_and_topic_info().topics
    if ground_truth_topic not in bag_topics.keys():
        raise NoiseEstimatorError(
            f"Ground truth topic\"{ground_truth_topic}\" not found in bag",
            f"Available topics: {bag_topics.keys()}")

    if bag_topics[ground_truth_topic].msg_type != "nav_msgs/Odometry":
        raise NoiseEstimatorError(
            f"Ground truth topic\"{ground_truth_topic} \
            \" is not of type nav_msgs/Odometry", f"Ground truth topic type: \
            {bag_topics[ground_truth_topic].msg_type}")

    return bag


def metrics(bag, topic, calculate_vectors, calculate_metrics, ground_truth,
            tf_buffer):
    # Initialize error_squared
    ground_truth_message = None
    bag_iterator = bag.read_messages(topics=[topic])
    while ground_truth_message is None:
        _, message, _ = next(bag_iterator)
        stamp = message.header.stamp
        ground_truth_message = ground_truth[stamp]
    variables_count = len(
        calculate_vectors(message, ground_truth_message, tf_buffer))
    # Preallocate memory
    vectors = np.empty((variables_count, 2, bag.get_message_count(topic)))
    samples = 0
    for _, message, _ in bag.read_messages(topics=[topic]):
        if rospy.is_shutdown():
            exit()
        stamp = message.header.stamp
        ground_truth_message = ground_truth[stamp]
        if ground_truth_message is not None:
            vectors[:, :,
                    samples] = calculate_vectors(message, ground_truth_message,
                                                 tf_buffer)
            samples += 1
    vectors = vectors[:, :, :samples]
    return calculate_metrics(vectors)


def odometryVectors(odom, ground_truth, tf_buffer):
    """Compare the odometry message to the ground truth message"""
    if odom.child_frame_id != "":
        twist_ground_truth = transform_odometry_child_frame(
            ground_truth, odom.child_frame_id, tf_buffer)
    else:
        twist_ground_truth = ground_truth
    return np.concatenate((poseVectors(odom.pose, ground_truth),
                           twistVectors(odom.twist, twist_ground_truth)))


def poseVectors(pose, ground_truth, *args, **kwargs):
    # pose can be stamped and with covariance, extract the pose
    while hasattr(pose, 'pose'):
        pose = pose.pose

    (pose_roll, pose_pitch, pose_yaw) = euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ])
    (gt_roll, gt_pitch, gt_yaw) = euler_from_quaternion([
        ground_truth.pose.pose.orientation.x,
        ground_truth.pose.pose.orientation.y,
        ground_truth.pose.pose.orientation.z,
        ground_truth.pose.pose.orientation.w
    ])

    return np.array([
        [pose.position.x, ground_truth.pose.pose.position.x],
        [pose.position.y, ground_truth.pose.pose.position.y],
        [pose.position.z, ground_truth.pose.pose.position.z],
        [pose_roll, gt_roll],
        [pose_pitch, gt_pitch],
        [pose_yaw, gt_yaw],
    ])


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
                transform.transform.rotation.x, transform.transform.rotation.y,
                transform.transform.rotation.z, transform.transform.rotation.w)
            if topic == '/tf_static':
                tf_buffer.set_transform_static(transform, "")
            else:
                tf_buffer.set_transform(transform, "")
    return tf_buffer


def transform_odometry_child_frame(msg, target_frame, tf_buffer):
    """Transform the child frame of the odom message to the target frame."""

    target_pose = transform_pose_child_frame(msg.pose.pose, target_frame,
                                             msg.child_frame_id, tf_buffer,
                                             msg.header.stamp)
    target_twist = transform_twist_child_frame(msg.twist.twist, target_frame,
                                               msg.child_frame_id, tf_buffer,
                                               msg.header.stamp)

    return Odometry(header=msg.header,
                    child_frame_id=target_frame,
                    pose=PoseWithCovariance(pose=target_pose),
                    twist=TwistWithCovariance(twist=target_twist))


def transform_pose_child_frame(pose, target_frame, child_frame, tf_buffer,
                               time):
    """Transform the child frame of the odom message to the target frame."""

    # Convert the pose to a PyKDL Frame
    child_to_parent_transform = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w),
        PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z))

    # Lookup the transform from the target frame to the child frame
    # and convert it to a PyKDL Frame
    # This is the same as the target pose in the child frame
    target_to_child_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(child_frame, target_frame, time))

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
    linear_velocity = PyKDL.Vector(twist.linear.x, twist.linear.y,
                                   twist.linear.z)
    angular_velocity = PyKDL.Vector(twist.angular.x, twist.angular.y,
                                    twist.angular.z)

    translation = PyKDL.Vector(
        transform[0, 3],
        transform[1, 3],
        transform[2, 3],
    )
    rotation = PyKDL.Rotation(
        transform[0, 0],
        transform[1, 0],
        transform[2, 0],
        transform[0, 1],
        transform[1, 1],
        transform[2, 1],
        transform[0, 2],
        transform[1, 2],
        transform[2, 2],
    )

    target_linear_velocity = rotation * \
        linear_velocity + angular_velocity * translation
    target_angular_velocity = rotation * angular_velocity

    return Twist(
        Vector3(target_linear_velocity[0], target_linear_velocity[1],
                target_linear_velocity[2]),
        Vector3(target_angular_velocity[0], target_angular_velocity[1],
                target_angular_velocity[2]))


def transform_twist_child_frame(twist, target_frame, child_frame, tf_buffer,
                                time):

    # Lookup the transform from the child frame to the target frame
    # and convert it to a PyKDL Frame
    child_to_target_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(target_frame, child_frame, time))

    twist = transform_twist(twist, child_to_target_transform)

    return twist


def twistVectors(twist, ground_truth, *args, **kwargs):
    # twist can be stamped and with covariance, extract the twist
    while hasattr(twist, 'twist'):
        twist = twist.twist

    return np.array([[twist.linear.x, ground_truth.twist.twist.linear.x],
                     [twist.linear.y, ground_truth.twist.twist.linear.y],
                     [twist.linear.z, ground_truth.twist.twist.linear.z],
                     [twist.angular.x, ground_truth.twist.twist.angular.x],
                     [twist.angular.y, ground_truth.twist.twist.angular.y],
                     [twist.angular.z, ground_truth.twist.twist.angular.z]])


def poseMetrics(vectors):
    error = np.concatenate((
        vectors[0:3, 0, :] - vectors[0:3, 1, :],
        angle_diff(vectors[3:6, 0, :], vectors[3:6, 1, :]),
    ))
    scale, bias = linear_fit(vectors)
    metrics = np.array([
        np.sqrt(np.mean(error**2, axis=1)),
        np.mean(error, axis=1),
        np.var(error, axis=1),
        np.max(np.abs(error), axis=1),
        np.concatenate((np.full([3], np.nan),
                        stats.circmean(error[3:6, :], np.pi, -np.pi, axis=1))),
        np.concatenate((np.full([3], np.nan),
                        stats.circvar(error[3:6, :], np.pi, -np.pi, axis=1))),
        scale,
        bias,
    ]).T
    return metrics


def twistMetrics(vectors):
    error = vectors[:, 0, :] - vectors[:, 1, :]
    scale, bias = linear_fit(vectors)
    metrics = np.array([
        np.sqrt(np.mean(error**2, axis=1)),
        np.mean(error, axis=1),
        np.var(error, axis=1),
        np.max(np.abs(error), axis=1),
        np.full([6], np.nan),
        np.full([6], np.nan),
        scale,
        bias,
    ]).T
    return metrics


def odometryMetrics(vectors):
    return np.concatenate(
        (poseMetrics(vectors[:6, :, :]), twistMetrics(vectors[6:, :, :])))


def imuMetrics(vectors):
    error = np.concatenate((
        angle_diff(vectors[0:3, 0, :], vectors[0:3, 1, :]),
        vectors[3:6, 0, :] - vectors[3:6, 1, :],
    ))
    scale, bias = linear_fit(vectors)
    metrics = np.array([
        np.sqrt(np.mean(error**2, axis=1)),
        np.mean(error, axis=1),
        np.var(error, axis=1),
        np.max(np.abs(error), axis=1),
        np.concatenate((stats.circmean(error[3:6, :], np.pi, -np.pi,
                                       axis=1), np.full([3], np.nan))),
        np.concatenate((stats.circvar(error[3:6, :], np.pi, -np.pi,
                                      axis=1), np.full([3], np.nan))),
        scale,
        bias,
    ]).T
    return metrics


def reprOdometryMetrics(metrics):
    return f"{reprPoseMetrics(metrics[:6, :])}\n" \
        f"{reprTwistMetrics(metrics[6:, :])}"


def reprPoseMetrics(metrics):
    return reprMetrics(metrics, ["x", "y", "z", "roll", "pitch", "yaw"])


def reprTwistMetrics(metrics):
    return reprMetrics(metrics, [
        "x velocity", "y velocity", "z velocity", "roll velocity",
        "pitch velocity", "yaw velocity"
    ])


def reprImuMetrics(metrics):
    return reprMetrics(metrics, [
        "roll", "pitch", "yaw", "roll velocity", "pitch velocity",
        "yaw velocity"
    ])


# Prints l-justified metrics, with labels on the left
def reprMetrics(metrics, labels):
    return '\n'.join([
        f"{label.ljust(20)}" + ''.join([
            f"{metric:.4E}".ljust(20) if not np.isnan(metric) else ''.ljust(20)
            for metric in metric_row
        ]) for label, metric_row in zip(labels, metrics)
    ])


# TODO: Prepare for yaml


def linear_fit(vectors):
    scale = np.zeros(vectors.shape[0])
    bias = np.zeros(vectors.shape[0])
    for i in range(vectors.shape[0]):
        # Ćheck for nan, inf and all zero
        if not np.any(np.isnan(vectors[i, :, :])) and not np.any(
                np.isinf(vectors[i, :, :])) and np.any(vectors[i,
                                                               0, :] >= 1e-5):
            with np.errstate(all='ignore'):  # Ignore errors just in case
                scale[i], bias[i] = np.polyfit(
                    vectors[i, 0, :],
                    vectors[i, 1, :],
                    1,
                )
        else:
            scale[i] = np.nan
            bias[i] = np.nan

    return scale, bias


localization_vectors = {
    'nav_msgs/Odometry': odometryVectors,
    'sensor_msgs/Imu': imuVectors,
    'geometry_msgs/PoseStamped': poseVectors,
    'geometry_msgs/PoseWithCovarianceStamped': poseVectors,
    'geometry_msgs/TwistStamped': twistVectors,
    'geometry_msgs/TwistWithCovarianceStamped': twistVectors
}
localization_metrics = {
    'nav_msgs/Odometry': odometryMetrics,
    'sensor_msgs/Imu': imuMetrics,
    'geometry_msgs/PoseStamped': poseMetrics,
    'geometry_msgs/PoseWithCovarianceStamped': poseMetrics,
    'geometry_msgs/TwistStamped': twistMetrics,
    'geometry_msgs/TwistWithCovarianceStamped': twistMetrics
}
repr_localization_metrics = {
    'nav_msgs/Odometry': reprOdometryMetrics,
    'sensor_msgs/Imu': reprImuMetrics,
    'geometry_msgs/PoseStamped': reprPoseMetrics,
    'geometry_msgs/PoseWithCovarianceStamped': reprPoseMetrics,
    'geometry_msgs/TwistStamped': reprTwistMetrics,
    'geometry_msgs/TwistWithCovarianceStamped': reprTwistMetrics
}

metrics_labels = [['RMSE', 'rmse'], ['Error mean', 'error_mean'],
                  ['Error variance', 'error_variance'],
                  ['Max error', 'error_max'],
                  ['Error circular mean', 'error_circular_mean'],
                  ['Error circular var.', 'error_circular_variance'],
                  ['Linear fit scale', 'linear_fit_scale'],
                  ['Linear fit bias', 'linear_fit_bias']]
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
    rospy.loginfo("\nLocalization topics found:" + ''.join([
        "\n\t" + topic.ljust(max_topic_length + 5) + message_type
        for topic, message_type in localization_topics.items()
    ]))

    ground_truth = StampedMessages(bag, ground_truth_topic)

    metrics_yaml = {}
    for topic, message_type in localization_topics.items():
        metrics_values = metrics(bag, topic,
                                 localization_vectors[message_type],
                                 localization_metrics[message_type],
                                 ground_truth, tf_buffer)
        metrics_yaml[topic] = {
            label[1]: metric.tolist()
            for label, metric in zip(metrics_labels, metrics_values.T)
        }
        rospy.loginfo(
            f"\tMetrics for {topic}\n" + "\t".ljust(21) +
            ''.join([label[0].ljust(20) for label in metrics_labels]) + "\n" +
            indent(repr_localization_metrics[message_type]
                   (metrics_values), "\t"))
        if rospy.is_shutdown():
            exit()

    if rospy.has_param("~output_file"):
        output_file = rospy.get_param("~output_file")
        with open(output_file, 'w') as file:
            yaml.dump(metrics_yaml, file)
            rospy.loginfo(f"Metrics saved to {output_file}")
