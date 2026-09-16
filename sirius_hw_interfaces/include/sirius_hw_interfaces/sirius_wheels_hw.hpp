#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace sirius_hw_interfaces
{

struct WheelJoint
{
  std::string ros_ctrl_name;   // name used by ros2_control (e.g. front_left_wheel_joint)
  std::string driver_ns;       // topic namespace (e.g. /wheels_left/motor0)

  double cmd_velocity{0.0};    // command sent to driver
  double state_position{0.0};  // feedback from driver
  double state_velocity{0.0};
  double state_effort{0.0};
};

// ---------------------------------------------------------------------------
// SiriusWheelsHW
//
// Topic-bridge hardware interface for the four ODrive-controlled wheels.
//
// For each wheel joint, it:
//   - Publishes velocity commands to `driver{driver_ns}/set_joint_state`
//     which is consumed by relaxing_middleware.
//   - Subscribes to `driver_raw{driver_ns}/joint_state` published by the
//     ODrive Python driver to get position/velocity/effort feedback.
//
// Configured via URDF <ros2_control> <hardware> <param> tags:
//   joint_names        (comma-separated, matches URDF joint names)
//   driver_namespaces  (comma-separated, matches ODrive driver motor paths)
// ---------------------------------------------------------------------------
class SiriusWheelsHW : public hardware_interface::SystemInterface
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
  std::vector<WheelJoint> wheels_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // publishers for velocity commands (one per wheel)
  std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> cmd_pubs_;

  // subscriptions for joint state feedback (one per wheel)
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> state_subs_;
};

}  // namespace sirius_hw_interfaces
