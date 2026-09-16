#!/usr/bin/env python3
"""Offline localization noise estimator — reads ROS2 bag files (rosbag2).

Usage:
    ros2 run sirius_navigation localization_evaluation.py \
        --ros-args -p bag_path:=/path/to/bag -p ground_truth:=/ground_truth
"""

import sys
import bisect
from collections import OrderedDict
from typing import Final
from textwrap import indent

import numpy as np
import scipy.stats as stats
import yaml
import PyKDL
import tf2_geometry_msgs
from geometry_msgs.msg import (PoseWithCovariance, TwistWithCovariance,
                                Pose, Twist, Vector3, Quaternion)
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

LOCALIZATION_TYPES: Final = {
    'nav_msgs/msg/Odometry',
    'sensor_msgs/msg/Imu',
    'geometry_msgs/msg/PoseWithCovarianceStamped',
    'geometry_msgs/msg/TwistWithCovarianceStamped',
}


class StampedMessages:

    def __init__(self, reader, topic):
        messages = OrderedDict()
        for msg, stamp in reader.read_messages(topics=[topic]):
            messages[stamp] = msg
        messages = OrderedDict(sorted(messages.items()))
        self.timestamps = list(messages.keys())
        self.messages = list(messages.values())

    def retrieve(self, timestamp):
        index = bisect.bisect(self.timestamps, timestamp)
        return self.messages[index - 1] if index else None

    def __getitem__(self, ts):
        return self.retrieve(ts)

    def __len__(self):
        return len(self.messages)


def angle_diff(a, b):
    d = b - a
    return (d + np.pi) % (2 * np.pi) - np.pi


def _euler(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])


def pose_vectors(pose, gt, *_):
    while hasattr(pose, 'pose'):
        pose = pose.pose
    pr, pp, py = _euler(pose.orientation)
    gr, gp, gy = _euler(gt.pose.pose.orientation)
    return np.array([
        [pose.position.x, gt.pose.pose.position.x],
        [pose.position.y, gt.pose.pose.position.y],
        [pose.position.z, gt.pose.pose.position.z],
        [pr, gr], [pp, gp], [py, gy],
    ])


def twist_vectors(twist, gt, *_):
    while hasattr(twist, 'twist'):
        twist = twist.twist
    t = gt.twist.twist
    return np.array([
        [twist.linear.x, t.linear.x],
        [twist.linear.y, t.linear.y],
        [twist.linear.z, t.linear.z],
        [twist.angular.x, t.angular.x],
        [twist.angular.y, t.angular.y],
        [twist.angular.z, t.angular.z],
    ])


def odometry_vectors(odom, gt, tf_buffer):
    return np.concatenate((pose_vectors(odom.pose, gt),
                           twist_vectors(odom.twist, gt)))


def imu_vectors(imu, gt, tf_buffer):
    ir, ip, iy = _euler(imu.orientation)
    gr, gp, gy = _euler(gt.pose.pose.orientation)
    return np.array([
        [ir, gr], [ip, gp], [iy, gy],
        [imu.angular_velocity.x, gt.twist.twist.angular.x],
        [imu.angular_velocity.y, gt.twist.twist.angular.y],
        [imu.angular_velocity.z, gt.twist.twist.angular.z],
    ])


def _linear_fit(vectors):
    scale = np.zeros(vectors.shape[0])
    bias = np.zeros(vectors.shape[0])
    for i in range(vectors.shape[0]):
        v = vectors[i]
        if (not np.any(np.isnan(v)) and not np.any(np.isinf(v)) and
                np.any(v[0] >= 1e-5)):
            with np.errstate(all='ignore'):
                scale[i], bias[i] = np.polyfit(v[0], v[1], 1)
        else:
            scale[i] = bias[i] = np.nan
    return scale, bias


def pose_metrics(vectors):
    error = np.concatenate((vectors[0:3, 0] - vectors[0:3, 1],
                             angle_diff(vectors[3:6, 0], vectors[3:6, 1])))
    scale, bias = _linear_fit(vectors)
    return np.array([
        np.sqrt(np.mean(error ** 2, axis=1)),
        np.mean(error, axis=1),
        np.var(error, axis=1),
        np.max(np.abs(error), axis=1),
        np.concatenate((np.full(3, np.nan),
                        stats.circmean(error[3:6], np.pi, -np.pi, axis=1))),
        np.concatenate((np.full(3, np.nan),
                        stats.circvar(error[3:6], np.pi, -np.pi, axis=1))),
        scale, bias,
    ]).T


def twist_metrics(vectors):
    error = vectors[:, 0] - vectors[:, 1]
    scale, bias = _linear_fit(vectors)
    return np.array([
        np.sqrt(np.mean(error ** 2, axis=1)),
        np.mean(error, axis=1),
        np.var(error, axis=1),
        np.max(np.abs(error), axis=1),
        np.full(6, np.nan), np.full(6, np.nan),
        scale, bias,
    ]).T


def odometry_metrics(v):
    return np.concatenate((pose_metrics(v[:6]), twist_metrics(v[6:])))


def imu_metrics(vectors):
    error = np.concatenate((
        angle_diff(vectors[0:3, 0], vectors[0:3, 1]),
        vectors[3:6, 0] - vectors[3:6, 1]))
    scale, bias = _linear_fit(vectors)
    return np.array([
        np.sqrt(np.mean(error ** 2, axis=1)),
        np.mean(error, axis=1),
        np.var(error, axis=1),
        np.max(np.abs(error), axis=1),
        np.concatenate((stats.circmean(error[:3], np.pi, -np.pi, axis=1),
                        np.full(3, np.nan))),
        np.concatenate((stats.circvar(error[:3], np.pi, -np.pi, axis=1),
                        np.full(3, np.nan))),
        scale, bias,
    ]).T


METRICS_LABELS = [
    ('RMSE', 'rmse'), ('Error mean', 'error_mean'),
    ('Error variance', 'error_variance'), ('Max error', 'error_max'),
    ('Circ. mean', 'error_circular_mean'), ('Circ. var.', 'error_circular_variance'),
    ('Scale', 'linear_fit_scale'), ('Bias', 'linear_fit_bias'),
]

LOCALIZATION_VECTORS = {
    'nav_msgs/msg/Odometry': odometry_vectors,
    'sensor_msgs/msg/Imu': imu_vectors,
    'geometry_msgs/msg/PoseWithCovarianceStamped': pose_vectors,
    'geometry_msgs/msg/TwistWithCovarianceStamped': twist_vectors,
}
LOCALIZATION_METRICS = {
    'nav_msgs/msg/Odometry': odometry_metrics,
    'sensor_msgs/msg/Imu': imu_metrics,
    'geometry_msgs/msg/PoseWithCovarianceStamped': pose_metrics,
    'geometry_msgs/msg/TwistWithCovarianceStamped': twist_metrics,
}


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = rclpy.create_node('noise_variance_estimator')

    node.declare_parameter('bag_path', '')
    node.declare_parameter('ground_truth', '/ground_truth')
    node.declare_parameter('output_file', '')

    bag_path = node.get_parameter('bag_path').value
    gt_topic = node.get_parameter('ground_truth').value
    output_file = node.get_parameter('output_file').value

    if not bag_path:
        node.get_logger().fatal('No bag_path provided')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        import rosidl_runtime_py.utilities as ru
    except ImportError as e:
        node.get_logger().fatal(f'Missing rosbag2_py: {e}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        t.name: t.type for t in reader.get_all_topics_and_types()
    }
    localization_topics = {
        t: tp for t, tp in topic_types.items()
        if tp in LOCALIZATION_TYPES and t != gt_topic
    }

    node.get_logger().info(f'Localization topics: {list(localization_topics.keys())}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
