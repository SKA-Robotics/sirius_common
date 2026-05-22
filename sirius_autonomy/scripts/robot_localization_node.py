#!/usr/bin/env python3

import rospy
import math
from typing import Tuple

# ROS imports
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry

# PyKDL imports
import PyKDL

# Geodetic transformations
import pyproj


class GPSConverter:
    """Convert GPS coordinates to local coordinate system using pyproj."""

    def __init__(
        self,
        origin_lat: float,
        origin_lon: float,
        reference_lat: float,
        reference_lon: float,
        reference_virtual_x: float,
        reference_virtual_y: float,
    ):
        """
        Initialize GPS converter with origin and reference points.

        Args:
            origin_lat, origin_lon: GPS coordinates of the origin (0,0) in virtual frame
            reference_lat, reference_lon: GPS coordinates of a reference point
            reference_virtual_x, reference_virtual_y: Virtual coordinates of the reference point
        """
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        self.reference_virtual_x = reference_virtual_x
        self.reference_virtual_y = reference_virtual_y

        # Initialize pyproj transformers
        self._initialize_transformers()

        # Calculate the rotation from GPS to virtual coordinates (scale is always 1)
        self._calculate_transform()

    def _initialize_transformers(self):
        """Initialize pyproj CRS and Transformers for local conversions."""
        proj_string = (
            f"+proj=tmerc +lat_0={self.origin_lat} +lon_0={self.origin_lon} "
            "+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        )

        self.local_crs = pyproj.CRS.from_proj4(proj_string)
        self.wgs84 = pyproj.CRS.from_epsg(4326)

        self.to_local = pyproj.Transformer.from_crs(
            self.wgs84, self.local_crs, always_xy=True
        )
        self.to_wgs84 = pyproj.Transformer.from_crs(
            self.local_crs, self.wgs84, always_xy=True
        )

        rospy.loginfo("pyproj transformers initialized")

    def _calculate_transform(self):
        """Compute rotation from local coordinates to virtual frame."""
        ref_x, ref_y = self.to_local.transform(self.reference_lon, self.reference_lat)
        origin_to_ref_bearing = math.atan2(ref_y, ref_x)
        virtual_bearing = math.atan2(self.reference_virtual_y, self.reference_virtual_x)
        self.rotation_angle = virtual_bearing - origin_to_ref_bearing
        rospy.loginfo(
            f"GPS->virtual rotation={math.degrees(self.rotation_angle):.2f}°, ref_local=({ref_x:.2f},{ref_y:.2f})"
        )

    def gps_to_virtual(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert (lat, lon) -> (virtual_x, virtual_y)."""
        local_x, local_y = self.to_local.transform(lon, lat)
        vec = PyKDL.Vector(local_x, local_y, 0.0)
        rot = PyKDL.Rotation.RotZ(self.rotation_angle)
        rotated_vec = rot * vec
        return float(rotated_vec[0]), float(rotated_vec[1])

    def virtual_to_gps(self, virtual_x: float, virtual_y: float) -> Tuple[float, float]:
        """Convert (virtual_x, virtual_y) -> (lat, lon)."""
        vec = PyKDL.Vector(virtual_x, virtual_y, 0.0)
        rot = PyKDL.Rotation.RotZ(-self.rotation_angle)
        rotated_vec = rot * vec
        local_x, local_y = float(rotated_vec[0]), float(rotated_vec[1])
        lon, lat = self.to_wgs84.transform(local_x, local_y)
        return lat, lon


class RobotLocalizationNode:
    """ROS node for robot localization using GPS and ArUco markers."""

    def __init__(self):
        rospy.init_node("robot_localization_node", anonymous=True)

        # Load entire config as a single dictionary from ROS param
        if not rospy.has_param("~config"):
            rospy.logerr(
                "No config parameter set! Please load the config YAML using rosparam."
            )
            raise RuntimeError("No config parameter set!")
        self.config = rospy.get_param("~config")

        # Initialize GPS converter
        self.gps_converter = GPSConverter(
            self.config["origin_lat"],
            self.config["origin_lon"],
            self.config["reference_lat"],
            self.config["reference_lon"],
            self.config["reference_virtual_x"],
            self.config["reference_virtual_y"],
        )

        # Load known marker positions in virtual coordinate system
        self.known_markers = self.config["known_markers"]

        # State variables
        self.latest_gps = None
        self.latest_markers = {}
        self.robot_pose = None

        # Publishers
        self.pose_pub = rospy.Publisher(
            "/robot_pose", PoseWithCovarianceStamped, queue_size=10
        )
        # Publish odometry on the topic expected by EKF config
        self.odom_pub = rospy.Publisher(
            "/robot_localization/odometry", Odometry, queue_size=10
        )

        # Subscribers
        self.gps_sub = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        self.markers_sub = rospy.Subscriber(
            "/aruco_markers", MarkerArray, self.markers_callback
        )

        # Timer for pose estimation
        self.pose_timer = rospy.Timer(rospy.Duration(0.1), self.estimate_pose)

        rospy.loginfo("Robot localization node initialized")

    # _load_config_from_rosparam is no longer needed; config is loaded as a single param

    # _get_default_config is no longer needed since defaults are handled in _load_config_from_rosparam

    def gps_callback(self, msg: NavSatFix):
        """Callback for GPS data."""
        if msg.status.status >= 0:  # Valid GPS fix
            self.latest_gps = msg
            rospy.logdebug(
                f"Received GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}"
            )

    def markers_callback(self, msg: MarkerArray):
        """Callback for ArUco marker detections."""
        self.latest_markers.clear()

        for marker in msg.markers:
            if marker.ns == "aruco_tags":  # Only process tag markers, not text
                marker_id = marker.id
                if marker_id in self.known_markers:
                    # Store marker pose in base_link frame
                    self.latest_markers[marker_id] = {
                        "pose": marker.pose,
                        "timestamp": marker.header.stamp,
                    }
                    rospy.logdebug(f"Detected known marker {marker_id}")

    def estimate_pose(self, event):
        """Estimate robot pose using GPS and ArUco markers."""
        if self.latest_gps is None:
            return

        # Get robot position from GPS
        robot_x, robot_y = self.gps_converter.gps_to_virtual(
            self.latest_gps.latitude, self.latest_gps.longitude
        )

        # Estimate orientation from ArUco markers
        robot_yaw = self._estimate_orientation_from_markers(robot_x, robot_y)

        # Create robot pose
        self.robot_pose = self._create_pose(robot_x, robot_y, robot_yaw)

        # Publish pose
        self._publish_pose()
        self._publish_odometry()

    def _estimate_orientation_from_markers(
        self, robot_x: float, robot_y: float
    ) -> float:
        """Estimate robot orientation using detected ArUco markers."""
        if not self.latest_markers:
            return 0.0  # Default orientation if no markers detected

        # Calculate expected vs observed marker positions
        orientation_estimates = []

        for marker_id, marker_data in self.latest_markers.items():
            if marker_id in self.known_markers:
                known_pos = self.known_markers[marker_id]
                observed_pose = marker_data["pose"]

                # Calculate expected marker position relative to robot
                expected_rel_x = known_pos[0] - robot_x
                expected_rel_y = known_pos[1] - robot_y

                # Calculate observed marker position relative to robot (in base_link frame)
                observed_rel_x = observed_pose.position.x
                observed_rel_y = observed_pose.position.y

                # Calculate orientation difference
                expected_angle = math.atan2(expected_rel_y, expected_rel_x)
                observed_angle = math.atan2(observed_rel_y, observed_rel_x)

                # The difference gives us the robot's orientation
                orientation_diff = expected_angle - observed_angle

                # Normalize to [-pi, pi]
                while orientation_diff > math.pi:
                    orientation_diff -= 2 * math.pi
                while orientation_diff < -math.pi:
                    orientation_diff += 2 * math.pi

                orientation_estimates.append(orientation_diff)

        if orientation_estimates:
            # Use median for robustness against outliers
            orientation_estimates.sort()
            median_index = len(orientation_estimates) // 2
            if len(orientation_estimates) % 2 == 0:
                robot_yaw = (
                    orientation_estimates[median_index - 1]
                    + orientation_estimates[median_index]
                ) / 2
            else:
                robot_yaw = orientation_estimates[median_index]

            rospy.logdebug(
                f"Estimated orientation from {len(orientation_estimates)} markers: {math.degrees(robot_yaw):.2f}°"
            )
            return robot_yaw
        else:
            return 0.0

    def _create_pose(self, x: float, y: float, yaw: float) -> Pose:
        """Create a Pose message from position and orientation."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # Convert yaw to quaternion
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def _publish_pose(self):
        """Publish robot pose."""
        if self.robot_pose is None:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose = self.robot_pose

        # Set covariance (GPS position is accurate, orientation less so)
        covariance = [0] * 36
        # Position covariance (GPS accuracy)
        covariance[0] = 0.1**2  # x variance
        covariance[7] = 0.1**2  # y variance
        covariance[14] = 0.1**2  # z variance
        # Orientation covariance
        covariance[21] = 0.1**2  # roll variance
        covariance[28] = 0.1**2  # pitch variance
        covariance[35] = 0.2**2  # yaw variance (less accurate from markers)

        pose_msg.pose.covariance = covariance
        self.pose_pub.publish(pose_msg)

    def _publish_odometry(self):
        """Publish robot odometry."""
        if self.robot_pose is None:
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = self.robot_pose

        # Set covariance
        covariance = [0] * 36
        covariance[0] = 0.1**2  # x variance
        covariance[7] = 0.1**2  # y variance
        covariance[14] = 0.1**2  # z variance
        covariance[21] = 0.1**2  # roll variance
        covariance[28] = 0.1**2  # pitch variance
        covariance[35] = 0.2**2  # yaw variance

        odom_msg.pose.covariance = covariance
        odom_msg.twist.covariance = [0] * 36  # No velocity information

        self.odom_pub.publish(odom_msg)


if __name__ == "__main__":
    try:
        node = RobotLocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
