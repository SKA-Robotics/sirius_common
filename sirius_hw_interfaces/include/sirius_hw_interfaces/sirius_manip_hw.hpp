#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace sirius_hw_interfaces
{

struct ManipJoint
{
  std::string name;
  double cmd_position{0.0};
  double cmd_velocity{0.0};
  double cmd_effort{0.0};
  double state_position{0.0};
  double state_velocity{0.0};
  double state_effort{0.0};
};

// ---------------------------------------------------------------------------
// SiriusManipHW
//
// Topic-bridge hardware interface for the Sirius rover manipulator.
//
// Communicates with the existing Python drivers (sirius_roboszpon_driver,
// sirius_moteus_driver) via standard sensor_msgs/JointState topics:
//   - Publishes commands to `command_topic` (default: /manip/set_joint_states)
//   - Subscribes to `state_topic` (default: /manip/joint_states)
//
// Configured via URDF <ros2_control> <hardware> <param> tags:
//   state_topic    Topic for reading joint states (from Python drivers)
//   command_topic  Topic for writing joint commands (to Python drivers)
// ---------------------------------------------------------------------------
class SiriusManipHW : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<ManipJoint> joints_;
  std::unordered_map<std::string, size_t> joint_index_;

  std::string state_topic_;
  std::string command_topic_;

  mutable std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_state_;
  bool state_received_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
};

}  // namespace sirius_hw_interfaces
