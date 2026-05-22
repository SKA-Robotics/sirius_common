#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading
import time
import yaml
from collections import defaultdict
from typing import Dict, List, Optional

# ROS imports
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

# PyKDL imports
import PyKDL


class CameraStream:
    """Class to handle individual camera stream connections and processing."""

    def __init__(self, camera_id: str, config: dict):
        self.camera_id = camera_id
        self.config = config
        self.rtsp_url = config["rtsp_url"]
        self.position = np.array(config["position"])
        self.orientation = np.array(config["orientation"])  # roll, pitch, yaw
        self.camera_matrix = np.array(config["camera_matrix"]).reshape(3, 3)
        self.dist_coeffs = np.array(config["dist_coeffs"])
        self.frame_id = config["frame_id"]

        self.cap = None
        self.is_connected = False
        self.latest_frame = None
        self.lock = threading.Lock()

        # Camera to base_link transformation frame
        self.camera_to_base_frame = self._compute_camera_transform()

        # Cache for rectification parameters (computed once)
        self._new_camera_matrix = None
        self._roi = None
        self._mapx = None
        self._mapy = None

    def _compute_camera_transform(self) -> PyKDL.Frame:
        """Compute transformation frame from camera frame to base_link using PyKDL."""
        # Convert roll, pitch, yaw to PyKDL rotation
        roll, pitch, yaw = self.orientation

        # Create rotation from roll, pitch, yaw (ZYX convention)
        rot = (PyKDL.Rotation.RotZ(yaw) * PyKDL.Rotation.RotY(pitch) *
               PyKDL.Rotation.RotX(roll))

        # Create vector for position
        pos = PyKDL.Vector(self.position[0], self.position[1],
                           self.position[2])

        # Create frame
        frame = PyKDL.Frame(rot, pos)

        return frame

    def connect(self) -> bool:
        """Connect to RTSP stream."""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if self.cap.isOpened():
                self.is_connected = True
                rospy.loginfo(f"Connected to camera {self.camera_id}")
                return True
            else:
                rospy.logwarn(f"Failed to connect to camera {self.camera_id}")
                return False
        except Exception as e:
            rospy.logerr(f"Error connecting to camera {self.camera_id}: {e}")
            return False

    def disconnect(self):
        """Disconnect from RTSP stream."""
        if self.cap:
            self.cap.release()
        self.is_connected = False
        self.latest_frame = None

    def read_frame(self) -> Optional[np.ndarray]:
        """Read and return the latest frame from the camera."""
        if not self.is_connected or not self.cap:
            return None

        try:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
                return frame
            else:
                self.is_connected = False
                return None
        except Exception as e:
            rospy.logwarn(
                f"Error reading frame from camera {self.camera_id}: {e}")
            self.is_connected = False
            return None

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the latest frame without reading a new one."""
        with self.lock:
            return self.latest_frame.copy(
            ) if self.latest_frame is not None else None

    def _initialize_rectification_maps(self, image_width: int,
                                       image_height: int):
        """Initialize rectification maps for the given image dimensions."""
        if self._mapx is not None and self._mapy is not None:
            return  # Already initialized

        # Compute optimal new camera matrix
        self._new_camera_matrix, self._roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            (image_width, image_height),
            1,
            (image_width, image_height),
        )

        # Compute rectification maps
        self._mapx, self._mapy = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self._new_camera_matrix,
            (image_width, image_height),
            cv2.CV_32FC1,
        )

        rospy.loginfo(
            f"Initialized rectification maps for camera {self.camera_id} ({image_width}x{image_height})"
        )

    def rectify_image(self, image: np.ndarray) -> np.ndarray:
        """Rectify image using camera intrinsics with cached maps."""
        if image is None:
            return None

        h, w = image.shape[:2]

        # Initialize rectification maps if not done yet
        if self._mapx is None or self._mapy is None:
            self._initialize_rectification_maps(w, h)

        # Use cached maps for fast rectification
        rectified = cv2.remap(image, self._mapx, self._mapy, cv2.INTER_LINEAR)

        # Apply ROI if needed
        if self._roi is not None:
            x, y, w_roi, h_roi = self._roi
            rectified = rectified[y:y + h_roi, x:x + w_roi]

        return rectified


class ArUcoDetector:
    """Class to handle ArUco tag detection and pose estimation."""

    def __init__(self, config: dict):
        self.config = config
        self.marker_size = config["marker_size"]
        self.max_distance = config["max_marker_distance"]
        self.confidence_threshold = config["confidence_threshold"]

        # Initialize ArUco dictionary (cached)
        dictionary_id = getattr(cv2.aruco, config["dictionary_id"])
        # Use newer OpenCV API getPredefinedDictionary instead of deprecated Dictionary_get
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Create ArUco detector (cached)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict,
                                                      self.aruco_params)

    def detect_tags(self, image: np.ndarray,
                    camera_matrix: np.ndarray) -> List[Dict]:
        """Detect ArUco tags in the image and estimate their poses."""
        if image is None:
            return []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers using cached detector
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        detections = []

        if ids is not None:
            for i, corner in enumerate(corners):
                tag_id = ids[i][0]

                # Estimate pose
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corner, self.marker_size, camera_matrix, None)

                if rvec is not None and tvec is not None:
                    # Check distance
                    distance = np.linalg.norm(tvec[0][0])
                    if distance <= self.max_distance:
                        # Convert rotation vector to rotation matrix
                        R, _ = cv2.Rodrigues(rvec[0][0])

                        # Create PyKDL frame from rotation matrix and translation vector
                        rot = PyKDL.Rotation(
                            R[0, 0],
                            R[0, 1],
                            R[0, 2],
                            R[1, 0],
                            R[1, 1],
                            R[1, 2],
                            R[2, 0],
                            R[2, 1],
                            R[2, 2],
                        )
                        pos = PyKDL.Vector(tvec[0][0][0], tvec[0][0][1],
                                           tvec[0][0][2])
                        camera_frame = PyKDL.Frame(rot, pos)

                        detections.append({
                            "id": tag_id,
                            "pose_camera": camera_frame,
                            "distance": distance,
                            "corners": corner[0],
                        })

        return detections

    def transform_to_base_link(
            self, pose_camera: PyKDL.Frame,
            camera_to_base_frame: PyKDL.Frame) -> PyKDL.Frame:
        """Transform pose from camera frame to base_link frame using PyKDL."""
        # Transform using PyKDL
        return camera_to_base_frame * pose_camera

    def _pykdl_frame_to_ros_pose(self, frame: PyKDL.Frame) -> Pose:
        """Convert PyKDL frame to ROS Pose message."""
        pose = Pose()
        pos = frame.p
        pose.position.x, pose.position.y, pose.position.z = pos[0], pos[
            1], pos[2]
        q = frame.M.GetQuaternion()
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = (q[0], q[1], q[2], q[3])
        return pose


class ArUcoDetectionNode:
    """Main ROS node for ArUco tag detection from multiple camera streams."""

    def __init__(self):
        rospy.init_node("aruco_detection_node", anonymous=True)

        # Load configuration
        self.config = self._load_config()

        # Initialize components
        self.detector = ArUcoDetector(self.config["aruco_params"])
        self.cameras = {}

        # Initialize camera streams
        self._initialize_cameras()

        # Publishers
        self.marker_pub = rospy.Publisher("/aruco_markers",
                                          MarkerArray,
                                          queue_size=10)

        # Threading
        self.camera_threads = {}
        self.running = True

        # Start camera threads
        self._start_camera_threads()

        # Main processing loop
        self._main_loop()

    def _load_config(self) -> dict:
        """Load configuration from YAML file."""
        config_path = rospy.get_param("~config_path",
                                      "config/camera_config.yaml")

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            rospy.loginfo(f"Loaded configuration from {config_path}")
            return config
        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            rospy.signal_shutdown("Failed to load configuration")
            return {}

    def _initialize_cameras(self):
        """Initialize all camera streams."""
        for camera_id, camera_config in self.config["cameras"].items():
            camera = CameraStream(camera_id, camera_config)
            if camera.connect():
                self.cameras[camera_id] = camera
                # Pre-initialize rectification maps if we can get frame dimensions
                self._pre_initialize_rectification(camera)
            else:
                rospy.logwarn(f"Failed to initialize camera {camera_id}")

    def _pre_initialize_rectification(self, camera: CameraStream):
        """Pre-initialize rectification maps for a camera."""
        try:
            frame = camera.read_frame()
            if frame is not None:
                h, w = frame.shape[:2]
                camera._initialize_rectification_maps(w, h)
                rospy.loginfo(
                    f"Pre-initialized rectification for camera {camera.camera_id}"
                )
            else:
                default_width = self.config["node_params"]["image_width"]
                default_height = self.config["node_params"]["image_height"]
                camera._initialize_rectification_maps(default_width,
                                                      default_height)
                rospy.loginfo(
                    f"Pre-initialized rectification for camera {camera.camera_id} with default dimensions"
                )
        except Exception as e:
            rospy.logwarn(
                f"Could not pre-initialize rectification for camera {camera.camera_id}: {e}"
            )

    def _start_camera_threads(self):
        """Start threads for each camera to continuously read frames."""
        for camera_id, camera in self.cameras.items():
            thread = threading.Thread(target=self._camera_loop,
                                      args=(camera, ))
            thread.daemon = True
            thread.start()
            self.camera_threads[camera_id] = thread

    def _camera_loop(self, camera: CameraStream):
        """Continuous loop for reading camera frames."""
        reconnect_interval = self.config["node_params"]["reconnect_interval"]

        while self.running:
            if not camera.is_connected:
                rospy.loginfo(
                    f"Attempting to reconnect to camera {camera.camera_id}")
                if camera.connect():
                    rospy.loginfo(f"Reconnected to camera {camera.camera_id}")
                else:
                    rospy.logwarn(
                        f"Failed to reconnect to camera {camera.camera_id}")
                    time.sleep(reconnect_interval)
                    continue

            frame = camera.read_frame()
            if frame is None:
                time.sleep(0.1)
                continue

            time.sleep(1.0 / self.config["node_params"]["publish_rate"])

    def _main_loop(self):
        """Main processing loop for ArUco detection and publishing."""
        rate = rospy.Rate(self.config["node_params"]["publish_rate"])

        while not rospy.is_shutdown() and self.running:
            try:
                all_detections = []

                for camera_id, camera in self.cameras.items():
                    if not camera.is_connected:
                        continue

                    frame = camera.get_latest_frame()
                    if frame is None:
                        continue

                    rectified_frame = camera.rectify_image(frame)
                    if rectified_frame is None:
                        continue

                    detections = self.detector.detect_tags(
                        rectified_frame, camera.camera_matrix)

                    for detection in detections:
                        pose_base = self.detector.transform_to_base_link(
                            detection["pose_camera"],
                            camera.camera_to_base_frame)

                        all_detections.append({
                            "id": detection["id"],
                            "pose": pose_base,
                            "distance": detection["distance"],
                            "camera_id": camera_id,
                        })

                self._publish_markers(all_detections)
                rate.sleep()

            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                rate.sleep()

    def _publish_markers(self, detections: List[Dict]):
        """Publish visualization markers for detected ArUco tags."""
        marker_array = MarkerArray()
        marker_array.header.stamp = rospy.Time.now()
        marker_array.header.frame_id = "base_link"

        # Group detections by tag ID to handle multiple detections of the same tag
        tag_groups = defaultdict(list)
        for detection in detections:
            tag_groups[detection["id"]].append(detection)

        for tag_id, tag_detections in tag_groups.items():
            best_detection = min(tag_detections, key=lambda x: x["distance"])
            pose_frame = best_detection["pose"]

            # Create cube marker
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "base_link"
            marker.ns = "aruco_tags"
            marker.id = tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = self._pykdl_frame_to_ros_pose(pose_frame)
            marker.scale.x = marker.scale.y = 0.05
            marker.scale.z = 0.01

            confidence = 1.0 / (1.0 + best_detection["distance"])
            if confidence > 0.8:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            elif confidence > 0.6:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(1.0)

            # Create text marker
            text_marker = Marker()
            text_marker.header.stamp = rospy.Time.now()
            text_marker.header.frame_id = "base_link"
            text_marker.ns = "aruco_text"
            text_marker.id = tag_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose_frame.p[0]
            text_marker.pose.position.y = pose_frame.p[1]
            text_marker.pose.position.z = pose_frame.p[2] + 0.1
            text_marker.pose.orientation.w = 1.0
            text_marker.text = f"ID: {tag_id}"
            text_marker.scale.z = 0.05
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(1.0)

            marker_array.markers.extend([marker, text_marker])

        self.marker_pub.publish(marker_array)

    def shutdown(self):
        """Cleanup on shutdown."""
        self.running = False

        # Disconnect all cameras
        for camera in self.cameras.values():
            camera.disconnect()

        # Clear caches
        for camera in self.cameras.values():
            camera._mapx = None
            camera._mapy = None

        rospy.loginfo("ArUco detection node shutdown complete")


if __name__ == "__main__":
    try:
        node = ArUcoDetectionNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
