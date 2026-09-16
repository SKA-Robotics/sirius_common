#include "sirius_hw_interfaces/sirius_wheels_hw.hpp"

#include <sstream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sirius_hw_interfaces
{

// ---- helpers ---------------------------------------------------------------

static std::vector<std::string> split(const std::string & s, char delim)
{
  std::vector<std::string> result;
  std::istringstream ss(s);
  std::string token;
  while (std::getline(ss, token, delim)) {
    if (!token.empty()) {
      result.push_back(token);
    }
  }
  return result;
}

// ---- on_init ---------------------------------------------------------------

hardware_interface::CallbackReturn SiriusWheelsHW::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read driver_namespaces from URDF <hardware><param> tags.
  // If not provided, fall back to the joint names declared in <ros2_control>.
  std::vector<std::string> driver_nss;
  if (info_.hardware_parameters.count("driver_namespaces")) {
    driver_nss = split(info_.hardware_parameters.at("driver_namespaces"), ',');
    // trim whitespace from each entry
    for (auto & ns : driver_nss) {
      ns.erase(0, ns.find_first_not_of(" \t\n\r"));
      ns.erase(ns.find_last_not_of(" \t\n\r") + 1);
    }
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    WheelJoint w;
    w.ros_ctrl_name = info_.joints[i].name;
    w.driver_ns = (i < driver_nss.size()) ? driver_nss[i] : ("/" + w.ros_ctrl_name);
    wheels_.push_back(w);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- export interfaces -----------------------------------------------------

std::vector<hardware_interface::StateInterface> SiriusWheelsHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (auto & w : wheels_) {
    si.emplace_back(w.ros_ctrl_name,
      hardware_interface::HW_IF_POSITION, &w.state_position);
    si.emplace_back(w.ros_ctrl_name,
      hardware_interface::HW_IF_VELOCITY, &w.state_velocity);
    si.emplace_back(w.ros_ctrl_name,
      hardware_interface::HW_IF_EFFORT, &w.state_effort);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> SiriusWheelsHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (auto & w : wheels_) {
    ci.emplace_back(w.ros_ctrl_name,
      hardware_interface::HW_IF_VELOCITY, &w.cmd_velocity);
  }
  return ci;
}

// ---- on_activate -----------------------------------------------------------

hardware_interface::CallbackReturn SiriusWheelsHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("sirius_wheels_hw");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  cmd_pubs_.clear();
  state_subs_.clear();

  for (size_t i = 0; i < wheels_.size(); ++i) {
    const std::string & ns = wheels_[i].driver_ns;

    // Commands go via relaxing_middleware: driver{ns}/set_joint_state
    std::string cmd_topic = "driver" + ns + "/set_joint_state";
    cmd_pubs_.push_back(
      node_->create_publisher<sensor_msgs::msg::JointState>(cmd_topic, 1));

    // State comes from the raw ODrive driver: driver_raw{ns}/joint_state
    std::string state_topic = "driver_raw" + ns + "/joint_state";
    auto & wheel = wheels_[i];
    state_subs_.push_back(
      node_->create_subscription<sensor_msgs::msg::JointState>(
        state_topic, 1,
        [&wheel](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (!msg->position.empty()) {
            wheel.state_position = msg->position[0];
          }
          if (!msg->velocity.empty()) {
            wheel.state_velocity = msg->velocity[0];
          }
          if (!msg->effort.empty()) {
            wheel.state_effort = msg->effort[0];
          }
        }));
  }

  executor_thread_ = std::thread([this]() { executor_->spin(); });
  RCLCPP_INFO(node_->get_logger(), "SiriusWheelsHW activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- on_deactivate ---------------------------------------------------------

hardware_interface::CallbackReturn SiriusWheelsHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero velocity to stop the wheels
  for (size_t i = 0; i < wheels_.size(); ++i) {
    sensor_msgs::msg::JointState msg;
    msg.name = {"motor"};
    msg.velocity = {0.0};
    cmd_pubs_[i]->publish(msg);
  }

  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  node_.reset();
  executor_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- read/write ------------------------------------------------------------

hardware_interface::return_type SiriusWheelsHW::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // State is updated asynchronously in the subscription callbacks.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SiriusWheelsHW::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < wheels_.size(); ++i) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.name = {"motor"};
    msg.velocity = {wheels_[i].cmd_velocity};
    cmd_pubs_[i]->publish(msg);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace sirius_hw_interfaces

PLUGINLIB_EXPORT_CLASS(
  sirius_hw_interfaces::SiriusWheelsHW,
  hardware_interface::SystemInterface)
