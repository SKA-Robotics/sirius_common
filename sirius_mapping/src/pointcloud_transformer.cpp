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

void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("pointcloud recieved");

  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("base_link", "slam", ros::Time::now(), ros::Duration(3));
    Eigen::Isometry3d transform = tf2::transformToEigen(transformStamped);
    Eigen::Matrix4f transform_matrix = transform.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    pcl::transformPointCloud(input_cloud, output_cloud, transform_matrix);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header.frame_id = "base_link";
    publisher.publish(output_msg);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("no transfrom between map and base_link");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud");
  ros::NodeHandle n;

  tf2_ros::TransformListener tfListener(tfBuffer);

  subscriber = n.subscribe("input_pointcloud", 1000, pcdCallback);
  publisher = n.advertise<sensor_msgs::PointCloud2>("output_pointcloud", 5);

  ros::spin();

  return 0;
}
