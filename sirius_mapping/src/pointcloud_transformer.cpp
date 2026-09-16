#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class PointcloudTransformerNode : public rclcpp::Node
{
public:
  PointcloudTransformerNode()
  : Node("pointcloud_transformer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("source_frame", "slam");
    this->declare_parameter("target_frame", "base_link_local");
    this->declare_parameter("output_frame", "base_link");

    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    output_frame_ = this->get_parameter("output_frame").as_string();

    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_pointcloud", 1000,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcdCallback(msg);
      });

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output_pointcloud", 5);
  }

private:
  void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero,
        tf2::durationFromSec(3.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
        "No transform between %s and %s: %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped);
    Eigen::Matrix4f transform_matrix = transform.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    pcl::fromROSMsg(*msg, input_cloud);
    pcl::transformPointCloud(input_cloud, output_cloud, transform_matrix);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header.frame_id = output_frame_;
    publisher_->publish(output_msg);
  }

  std::string source_frame_;
  std::string target_frame_;
  std::string output_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
