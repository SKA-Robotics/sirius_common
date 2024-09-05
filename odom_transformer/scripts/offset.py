from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
import rospy
import tf2_ros

class OdometryChildFrameTransformer():
    def __init__(self):

        rospy.init_node('odom_offset', anonymous=True)


        self.publish_tf = rospy.get_param('~publish_tf', False)
        self.translation = {'x': rospy.get_param('~pose/x', 0), 'y': rospy.get_param('~pose/y', 0), 'z': rospy.get_param('~pose/z', 0)}
        # self.orientation = {'x': rospy.get_param('~orientation/x', 0), 'y': rospy.get_param('~orientation/y', 0), 'z': rospy.get_param('~orientation/z', 0), 'w': rospy.get_param('~orientation/w', 0)}

        self.odom_sub = rospy.Subscriber('odom_in', Odometry, self.odom_callback)
        self.odom_pub = rospy.Publisher('odom_out', Odometry, queue_size=10)
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    def run(self):
        rospy.spin()

    def odom_callback(self, msg):
        transformed_odom = offset_odometry(msg, self.translation)
        self.odom_pub.publish(transformed_odom)
        if self.publish_tf:
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
    


def offset_odometry(msg, translation):
    """Add offset to the odom message."""

    target_pose = PoseWithCovariance(pose=msg.pose.pose)
    target_pose.pose.position.x += translation['x']
    target_pose.pose.position.y += translation['y']
    target_pose.pose.position.z += translation['z']

    return Odometry(header=msg.header,
                    child_frame_id=msg.child_frame_id,
                    pose=target_pose,
                    twist=TwistWithCovariance(twist=msg.twist.twist))

if __name__ == '__main__':
    try:
        OdometryChildFrameTransformer().run()
    except rospy.ROSInterruptException:
        pass