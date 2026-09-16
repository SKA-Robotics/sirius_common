#include "sirius_hw_interfaces/sirius_manip_hw.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sirius_hw_interfaces
{

// ---- on_init ---------------------------------------------------------------

hardware_interface::CallbackReturn SiriusManipHW::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  state_topic_ = "/manip/joint_states";
  command_topic_ = "/manip/set_joint_states";
  if (info_.hardware_parameters.count("state_topic")) {
    state_topic_ = info_.hardware_parameters.at("state_topic");
  }
  if (info_.hardware_parameters.count("command_topic")) {
    command_topic_ = info_.hardware_parameters.at("command_topic");
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ManipJoint j;
    j.name = info_.joints[i].name;
    joint_index_[j.name] = i;
    joints_.push_back(j);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- export interfaces -----------------------------------------------------

std::vector<hardware_interface::StateInterface> SiriusManipHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (auto & j : joints_) {
    si.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.state_position);
    si.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.state_velocity);
    si.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.state_effort);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> SiriusManipHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (auto & j : joints_) {
    ci.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd_position);
    ci.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.cmd_velocity);
    ci.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.cmd_effort);
  }
  return ci;
}

// ---- on_activate -----------------------------------------------------------

hardware_interface::CallbackReturn SiriusManipHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("sirius_manip_hw");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(command_topic_, 1);

  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = *msg;
      state_received_ = true;
    });

  executor_thread_ = std::thread([this]() { executor_->spin(); });
  RCLCPP_INFO(node_->get_logger(), "SiriusManipHW activated (state: %s, cmd: %s)",
    state_topic_.c_str(), command_topic_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- on_deactivate ---------------------------------------------------------

hardware_interface::CallbackReturn SiriusManipHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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

// ---- read ------------------------------------------------------------------

hardware_interface::return_type SiriusManipHW::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_received_) {
    return hardware_interface::return_type::OK;
  }

  const auto & msg = latest_state_;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = joint_index_.find(msg.name[i]);
    if (it == joint_index_.end()) {
      continue;
    }
    size_t idx = it->second;
    if (i < msg.position.size()) {
      joints_[idx].state_position = msg.position[i];
    }
    if (i < msg.velocity.size()) {
      joints_[idx].state_velocity = msg.velocity[i];
    }
    if (i < msg.effort.size()) {
      joints_[idx].state_effort = msg.effort[i];
    }
  }
  return hardware_interface::return_type::OK;
}

// ---- write -----------------------------------------------------------------

hardware_interface::return_type SiriusManipHW::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();
  for (const auto & j : joints_) {
    msg.name.push_back(j.name);
    msg.position.push_back(j.cmd_position);
    msg.velocity.push_back(j.cmd_velocity);
    msg.effort.push_back(j.cmd_effort);
  }
  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace sirius_hw_interfaces

PLUGINLIB_EXPORT_CLASS(
  sirius_hw_interfaces::SiriusManipHW,
  hardware_interface::SystemInterface)
