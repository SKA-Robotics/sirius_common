from geometry_msgs.msg import Vector3, Twist, Pose, PoseWithCovariance, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
import PyKDL
import rospy
import tf2_geometry_msgs
import tf2_ros

class OdometryChildFrameTransformer():
    def __init__(self):

        rospy.init_node('odom_child_frame_transformer', anonymous=True)


        self.target_frame = rospy.get_param('~target_frame')
        timeout = rospy.get_param('~timeout', 5)
        self.timeout = rospy.Duration(timeout)

        self.odom_sub = rospy.Subscriber('odom_in', Odometry, self.odom_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('odom_out', Odometry, queue_size=10)
    
    def run(self):
        rospy.spin()

    def odom_callback(self, msg):
        transformed_odom = transform_odometry_child_frame(msg, self.target_frame, self.tf_buffer, self.timeout)
        self.odom_pub.publish(transformed_odom)
        self.publish_odom_to_tf(transformed_odom)
    
    def publish_odom_to_tf(self, odom):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = odom.header.frame_id
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation.x = odom.pose.pose.orientation.x
        tf_msg.transform.rotation.y = odom.pose.pose.orientation.y
        tf_msg.transform.rotation.z = odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(tf_msg)
    


def transform_odometry_child_frame(msg, target_frame, tf_buffer, timeout):
    """Transform the child frame of the odom message to the target frame."""

    target_pose = transform_pose_child_frame(msg.pose.pose, target_frame,
                                             msg.child_frame_id, tf_buffer,
                                             msg.header.stamp, timeout)
    target_twist = transform_twist_child_frame(msg.twist.twist, target_frame,
                                               msg.child_frame_id, tf_buffer,
                                               msg.header.stamp, timeout)

    return Odometry(header=msg.header,
                    child_frame_id=target_frame,
                    pose=PoseWithCovariance(pose=target_pose),
                    twist=TwistWithCovariance(twist=target_twist))


def transform_pose_child_frame(pose, target_frame, child_frame, tf_buffer,
                               time, timeout):
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
        tf_buffer.lookup_transform(child_frame, target_frame, time, timeout))

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
                                time, timeout):

    # Lookup the transform from the child frame to the target frame
    # and convert it to a PyKDL Frame
    child_to_target_transform = tf2_geometry_msgs.transform_to_kdl(
        tf_buffer.lookup_transform(target_frame, child_frame, time, timeout))

    twist = transform_twist(twist, child_to_target_transform)

    return twist

if __name__ == '__main__':
    try:
        OdometryChildFrameTransformer().run()
    except rospy.ROSInterruptException:
        pass