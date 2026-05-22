# ArUco Detection and Robot Localization System

A ROS 1 Noetic system for detecting ArUco tags from multiple RTSP camera streams and using them along with GPS data for robot localization. The system consists of two main nodes:

1. **ArUco Detection Node**: Connects to 8 camera streams, rectifies images using predefined camera intrinsics, detects ArUco tags, and publishes their positions relative to the robot's base_link frame.

2. **Robot Localization Node**: Combines GPS coordinates with ArUco marker positions to determine robot location and orientation in a virtual coordinate system.

## Features

### ArUco Detection Node

-   **Multi-camera support**: Connects to 8 RTSP camera streams simultaneously
-   **Image rectification**: Uses predefined camera intrinsics to rectify distorted images
-   **ArUco tag detection**: Detects ArUco markers using OpenCV
-   **Pose estimation**: Estimates 3D pose of detected tags
-   **Configurable camera poses**: Each camera has configurable position and orientation relative to base_link
-   **Robust connection handling**: Automatic reconnection to failed camera streams
-   **PyKDL transforms**: Uses PyKDL for robust and standardized coordinate frame transformations
-   **Performance optimizations**: Caches rectification maps, ArUco detectors, and quaternion conversions for improved performance

### Robot Localization Node

-   **GPS integration**: Uses accurate GPS coordinates for robot position determination
-   **ArUco-based orientation**: Uses detected ArUco markers to determine robot orientation
-   **Virtual coordinate system**: Defines a local coordinate system based on GPS reference points
-   **Robust orientation estimation**: Uses median filtering for orientation estimation from multiple markers
-   **Pose publishing**: Publishes robot pose as PoseWithCovarianceStamped and Odometry messages
-   **Accurate geodetic transformations**: Uses pyproj for precise GPS coordinate conversions
-   **Bidirectional conversion**: Supports both GPS-to-virtual and virtual-to-GPS coordinate transformations

## Dependencies

-   ROS Noetic
-   Python 3
-   OpenCV 4.x
-   NumPy
-   PyYAML
-   cv_bridge
-   tf2_ros
-   PyKDL (orocos_kdl)
-   pyproj (for accurate GPS coordinate transformations)

## Installation

1. Clone this package into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone <repository_url> aruco_detection_node
```

2. Install Python dependencies:

```bash
pip3 install opencv-contrib-python numpy pyyaml pyproj
```

Note: PyKDL is typically installed with ROS Noetic, but if you encounter issues, you can install it separately:

```bash
sudo apt-get install python3-orocos-kdl
```

Note: pyproj can also be installed via apt:

```bash
sudo apt-get install python3-pyproj
```

3. Build the package:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Configuration

### ArUco Detection Node

Edit the `config/camera_config.yaml` file to configure your cameras:

```yaml
cameras:
    camera_0:
        rtsp_url: 'rtsp://username:password@192.168.1.100:554/stream1'
        position: [1.0, 0.0, 0.5] # x, y, z in meters relative to base_link
        orientation: [0.0, 0.0, 0.0] # roll, pitch, yaw in radians
        camera_matrix: [1000.0, 0.0, 640.0, 0.0, 1000.0, 480.0, 0.0, 0.0, 1.0]
        dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
        frame_id: 'camera_0_frame'
```

### Configuration Parameters

-   **rtsp_url**: RTSP stream URL for the camera
-   **position**: Camera position relative to base_link [x, y, z] in meters
-   **orientation**: Camera orientation relative to base_link [roll, pitch, yaw] in radians
-   **camera_matrix**: 3x3 camera intrinsic matrix (flattened)
-   **dist_coeffs**: Distortion coefficients [k1, k2, p1, p2, k3]
-   **frame_id**: Camera frame ID

### ArUco Parameters

-   **dictionary_id**: ArUco dictionary to use (e.g., "DICT_6X6_250")
-   **marker_size**: Size of ArUco markers in meters
-   **max_marker_distance**: Maximum distance to detect markers
-   **confidence_threshold**: Minimum confidence for detection

### Robot Localization Node

Edit the `config/localization_config.yaml` file to configure the localization system:

```yaml
# GPS Coordinate System Definition
origin_lat: 52.2297 # GPS latitude of origin (0,0) in virtual frame
origin_lon: 21.0122 # GPS longitude of origin (0,0) in virtual frame

# Reference point to define coordinate system
reference_lat: 52.2307 # GPS latitude of reference point
reference_lon: 21.0122 # GPS longitude of reference point
reference_virtual_x: 100.0 # Virtual X coordinate of reference point
reference_virtual_y: 0.0 # Virtual Y coordinate of reference point

# Known ArUco marker positions in virtual coordinate system
known_markers:
    0: [5.0, 5.0, 0.0] # Marker 0 at (5, 5, 0) in virtual frame
    1: [5.0, -5.0, 0.0] # Marker 1 at (5, -5, 0) in virtual frame
    # ... add more markers as needed
```

#### Configuration Parameters

-   **origin_lat/origin_lon**: GPS coordinates that become (0,0) in the virtual coordinate system
-   **reference_lat/reference_lon**: GPS coordinates of a reference point
-   **reference_virtual_x/reference_virtual_y**: Virtual coordinates of the reference point (defines scale and orientation)
-   **known_markers**: Dictionary mapping marker IDs to their positions in the virtual coordinate system

## Usage

### ArUco Detection Node

Launch the ArUco detection node:

```bash
roslaunch aruco_detection_node aruco_detection.launch
```

Or run directly:

```bash
rosrun aruco_detection_node aruco_detection_node.py
```

### Robot Localization Node

Launch the robot localization node:

```bash
roslaunch aruco_detection_node robot_localization.launch
```

Or run directly:

```bash
rosrun aruco_detection_node robot_localization_node.py
```

### Complete System

Launch both nodes together:

```bash
roslaunch aruco_detection_node complete_system.launch
```

## Topics

### ArUco Detection Node

#### Published Topics

-   `/aruco_markers` (visualization_msgs/MarkerArray): Visualization markers for RViz display

### Robot Localization Node

#### Subscribed Topics

-   `/gps/fix` (sensor_msgs/NavSatFix): GPS coordinates from GPS receiver
-   `/aruco_markers` (visualization_msgs/MarkerArray): ArUco marker detections from ArUco detection node

#### Published Topics

-   `/robot_pose` (geometry_msgs/PoseWithCovarianceStamped): Robot pose with covariance in map frame
-   `/robot_odom` (nav_msgs/Odometry): Robot odometry in map frame

## Coordinate Frames

### ArUco Detection Node

-   **base_link**: Robot's base frame (reference frame for all tag poses)
-   **camera_X_frame**: Individual camera frames

Tag positions are published relative to the base_link frame, taking into account each camera's position and orientation.

### Robot Localization Node

-   **map**: Virtual coordinate system defined by GPS reference points
-   **base_link**: Robot's base frame

The virtual coordinate system is defined by:

1. **Origin**: A GPS coordinate that becomes (0,0) in the virtual frame
2. **Reference point**: A second GPS coordinate and its corresponding virtual position to define scale and orientation

## GPS Coordinate Conversion with pyproj

The robot localization node uses `pyproj` for accurate GPS coordinate transformations:

### Benefits of pyproj

-   **Accurate Geodetic Calculations**: Uses proper ellipsoid models (WGS84) instead of spherical approximations
-   **Local Projection**: Creates a local Transverse Mercator projection centered at the origin for optimal accuracy
-   **Bidirectional Conversion**: Supports both GPS-to-virtual and virtual-to-GPS coordinate transformations
-   **Industry Standard**: Uses the same algorithms as professional GIS software
-   **Better Accuracy**: Significantly more accurate than Haversine formula for local coordinate systems

### How it Works

1. **Local Projection**: Creates a Transverse Mercator projection centered at the GPS origin point
2. **Coordinate Transformation**: Converts WGS84 GPS coordinates to local projection coordinates
3. **Scale and Rotation**: Applies scaling and rotation to match the virtual coordinate system
4. **Inverse Transformation**: Provides reverse conversion from virtual coordinates back to GPS

### Test Script

A test script is provided to demonstrate the accuracy:

```bash
python3 scripts/test_gps_conversion.py
```

## Performance Optimizations

The node includes several performance optimizations for real-time operation:

### Cached Rectification Maps

-   Rectification maps are computed once per camera and cached
-   Uses `cv2.remap()` instead of `cv2.undistort()` for faster image processing
-   Maps are pre-initialized during camera setup

### Cached ArUco Detection

-   ArUco detector objects are created once and reused
-   Eliminates repeated dictionary and parameter initialization

### PyKDL Transforms

-   Uses PyKDL for robust rotation matrix to quaternion conversion
-   PyKDL frames are computed once per camera configuration

### Memory Management

-   Caches are cleared on shutdown to prevent memory leaks
-   Efficient frame copying and thread-safe operations

## How Robot Localization Works

The robot localization system combines GPS and ArUco marker data to provide accurate pose estimation:

### GPS Position Estimation

1. **Coordinate System Definition**: A virtual coordinate system is defined using two GPS reference points
2. **GPS to Virtual Conversion**: Current GPS coordinates are converted to virtual coordinates using pyproj for accurate geodetic transformations
3. **Accurate Positioning**: GPS provides accurate position information (typically ±1-3 meters)
4. **Local Projection**: Uses a local Transverse Mercator projection centered at the origin for optimal accuracy

### ArUco-Based Orientation Estimation

1. **Marker Detection**: ArUco markers are detected by the ArUco detection node
2. **Known Positions**: The system uses pre-defined marker positions in the virtual coordinate system
3. **Orientation Calculation**: For each detected marker, the system calculates the expected vs observed marker position relative to the robot
4. **Robust Estimation**: Multiple marker detections are combined using median filtering for robust orientation estimation

### Pose Publishing

The system publishes:

-   **PoseWithCovarianceStamped**: Robot pose with uncertainty information
-   **Odometry**: Standard ROS odometry message for navigation systems

### Coordinate System Setup

To set up the coordinate system:

1. Choose a GPS coordinate as the origin (0,0) in your virtual frame
2. Choose a second GPS coordinate and define where it should be in your virtual frame
3. Place ArUco markers at known positions in your virtual coordinate system
4. Update the configuration file with these coordinates

## RViz Visualization

To visualize the detected ArUco tags in RViz:

1. Start RViz:

```bash
rosrun rviz rviz
```

2. Add a MarkerArray display:

    - Click "Add" → "By display type" → "MarkerArray"
    - Set the Marker Topic to `/aruco_markers`

3. Configure the Fixed Frame to `base_link`

The visualization includes:

-   **Cube markers**: Represent the detected ArUco tags with color-coded confidence levels
    -   Green: High confidence (>80%)
    -   Yellow: Medium confidence (60-80%)
    -   Red: Low confidence (<60%)
-   **Text labels**: Show the tag ID above each marker
-   **Auto-expiry**: Markers disappear after 1 second if not updated

## Troubleshooting

### Camera Connection Issues

-   Check RTSP URLs in the configuration file
-   Verify network connectivity to camera IP addresses
-   Ensure correct username/password for camera authentication

### No Tag Detections

-   Verify ArUco marker size matches configuration
-   Check camera intrinsics calibration
-   Ensure markers are within the maximum detection distance
-   Verify ArUco dictionary matches the markers being used

### Performance Issues

-   Reduce publish rate in configuration
-   Lower image resolution if needed
-   Increase confidence threshold to reduce false positives

## Customization

### Adding More Cameras

Add additional camera configurations to the YAML file following the same format.

### Changing ArUco Dictionary

Modify the `dictionary_id` parameter in the configuration file. Available dictionaries:

-   DICT_4X4_50
-   DICT_4X4_100
-   DICT_4X4_250
-   DICT_4X4_1000
-   DICT_5X5_50
-   DICT_5X5_100
-   DICT_5X5_250
-   DICT_5X5_1000
-   DICT_6X6_50
-   DICT_6X6_100
-   DICT_6X6_250
-   DICT_6X6_1000
-   DICT_7X7_50
-   DICT_7X7_100
-   DICT_7X7_250
-   DICT_7X7_1000

## License

MIT License
