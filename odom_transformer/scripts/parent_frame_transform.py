import PyKDL
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry


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
