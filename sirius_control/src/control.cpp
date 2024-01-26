#include <ros/ros.h>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sirius_control_loop");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  combined_robot_hw::CombinedRobotHW hw;
  bool init_success = hw.init(nh, nh);

  controller_manager::ControllerManager cm(&hw, nh);

  ros::Rate rate(10);  // 200Hz update rate

  ROS_INFO("sirius_control_loop started");
  while (ros::ok())
  {
    hw.read(ros::Time::now(), rate.expectedCycleTime());
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    hw.write(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }

  spinner.stop();
  return 0;
}