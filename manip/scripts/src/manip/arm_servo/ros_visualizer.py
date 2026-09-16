import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped


class RosVisualizer:

    def __init__(self, pose_goal_topic: str, node):
        self._node = node
        self._publisher = node.create_publisher(PoseStamped, pose_goal_topic, 10)

    def visualize_goal_pose(self, pose: np.ndarray, frame_id: str):
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = pose[0, 3]
        msg.pose.position.y = pose[1, 3]
        msg.pose.position.z = pose[2, 3]
        try:
            quat = R.from_matrix(pose[:3, :3]).as_quat()
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            self._publisher.publish(msg)
        except Exception as e:
            print(e)
