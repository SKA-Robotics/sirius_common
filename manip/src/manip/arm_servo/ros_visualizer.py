from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

class RosVisualizer:
    def __init__(self, pose_goal_topic: str):
        self.pose_goal_publisher = rospy.Publisher(pose_goal_topic, PoseStamped, queue_size=10)

    def visualize_goal_pose(self, pose: np.ndarray, frame_id: str):
        command = PoseStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = frame_id

        command.pose.position.x = pose[0, 3]
        command.pose.position.y = pose[1, 3]
        command.pose.position.z = pose[2, 3]

        rotation_matrix = pose[:3, :3]
        try:
            quat = R.from_matrix(rotation_matrix).as_quat()
            command.pose.orientation.x = quat[0]
            command.pose.orientation.y = quat[1]
            command.pose.orientation.z = quat[2]
            command.pose.orientation.w = quat[3]

            self.pose_goal_publisher.publish(command)
        except Exception as e:
            print(e)
