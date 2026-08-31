#include <cmath>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher publisher;
ros::Subscriber subscriber;
tf2_ros::Buffer tfBuffer;

std::string sourceFrame = "slam";
std::string targetFrame = "base_link_local";
std::string outputFrameId = "base_link";

// Clips points close to the edges of the camera's field of view, where depth
// readings tend to be noisy or invalid (e.g. stereo mismatches, lens
// distortion). The test is performed in `camera_frame`, which is expected to
// follow the standard ROS optical convention (Z forward, X right, Y down).
struct FrustumFilterParams
{
  bool enabled = false;
  std::string shape = "pyramid";  // "pyramid" (rectangular) or "cone"
  std::string camera_frame = "front_rgb_camera_optical";
  double horizontal_fov_deg = 60.0;  // pyramid: full angle kept around the optical axis
  double vertical_fov_deg = 45.0;    // pyramid: full angle kept around the optical axis
  double cone_fov_deg = 60.0;        // cone: full angle kept around the optical axis
  double min_range = 0.1;            // metres, along the optical axis (depth)
  double max_range = 5.0;            // metres, along the optical axis (depth)
} frustumParams;

bool isInsideFrustum(const pcl::PointXYZ& p)
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    return false;

  if (p.z < frustumParams.min_range || p.z > frustumParams.max_range)
    return false;

  if (frustumParams.shape == "cone")
  {
    double half_angle = 0.5 * frustumParams.cone_fov_deg * M_PI / 180.0;
    double angle_from_axis = std::atan2(std::hypot(p.x, p.y), p.z);
    return angle_from_axis <= half_angle;
  }

  // Pyramid: independent horizontal / vertical half-angle bounds.
  double half_h = 0.5 * frustumParams.horizontal_fov_deg * M_PI / 180.0;
  double half_v = 0.5 * frustumParams.vertical_fov_deg * M_PI / 180.0;
  double angle_h = std::atan2(p.x, p.z);
  double angle_v = std::atan2(p.y, p.z);
  return std::fabs(angle_h) <= half_h && std::fabs(angle_v) <= half_v;
}

void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("pointcloud recieved");

  try
  {
    geometry_msgs::TransformStamped transformStamped =
        tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time::now(), ros::Duration(3));
    Eigen::Isometry3d transform = tf2::transformToEigen(transformStamped);
    Eigen::Matrix4f transform_matrix = transform.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    if (frustumParams.enabled)
    {
      geometry_msgs::TransformStamped cameraTransformStamped = tfBuffer.lookupTransform(
          frustumParams.camera_frame, msg->header.frame_id, ros::Time::now(), ros::Duration(3));
      Eigen::Matrix4f camera_transform_matrix =
          tf2::transformToEigen(cameraTransformStamped).matrix().cast<float>();

      pcl::PointCloud<pcl::PointXYZ> camera_frame_cloud;
      pcl::transformPointCloud(input_cloud, camera_frame_cloud, camera_transform_matrix);

      pcl::PointCloud<pcl::PointXYZ> clipped_cloud;
      clipped_cloud.reserve(input_cloud.size());
      for (std::size_t i = 0; i < input_cloud.size(); ++i)
      {
        if (isInsideFrustum(camera_frame_cloud[i]))
          clipped_cloud.push_back(input_cloud[i]);
      }
      input_cloud.swap(clipped_cloud);
    }

    pcl::transformPointCloud(input_cloud, output_cloud, transform_matrix);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header.frame_id = outputFrameId;
    publisher.publish(output_msg);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("point cloud transform failed: %s", ex.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  tf2_ros::TransformListener tfListener(tfBuffer);

  pnh.param("target_frame", targetFrame, targetFrame);
  pnh.param("source_frame", sourceFrame, sourceFrame);
  pnh.param("output_frame_id", outputFrameId, outputFrameId);

  pnh.param("frustum_filter/enabled", frustumParams.enabled, frustumParams.enabled);
  pnh.param("frustum_filter/shape", frustumParams.shape, frustumParams.shape);
  pnh.param("frustum_filter/camera_frame", frustumParams.camera_frame, frustumParams.camera_frame);
  pnh.param("frustum_filter/horizontal_fov_deg", frustumParams.horizontal_fov_deg, frustumParams.horizontal_fov_deg);
  pnh.param("frustum_filter/vertical_fov_deg", frustumParams.vertical_fov_deg, frustumParams.vertical_fov_deg);
  pnh.param("frustum_filter/cone_fov_deg", frustumParams.cone_fov_deg, frustumParams.cone_fov_deg);
  pnh.param("frustum_filter/min_range", frustumParams.min_range, frustumParams.min_range);
  pnh.param("frustum_filter/max_range", frustumParams.max_range, frustumParams.max_range);

  subscriber = n.subscribe("input_pointcloud", 1000, pcdCallback);
  publisher = n.advertise<sensor_msgs::PointCloud2>("output_pointcloud", 5);

  ros::spin();

  return 0;
}
