#include "ros_control_controllers/mit_joint_controller.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace ros_control_controllers {

controller_interface::CallbackReturn MitJointController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<double>("command_timeout", 0.5);
  auto_declare<double>("damping_kd", 0.0);

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  damping_kd_ = get_node()->get_parameter("damping_kd").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    config.names.push_back(joint + "/kp");
    config.names.push_back(joint + "/kd");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MitJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::CallbackReturn MitJointController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  const std::size_t n = joint_names_.size();

  joint_index_.clear();
  for (std::size_t i = 0; i < n; ++i) {
    joint_index_[joint_names_[i]] = i;
  }

  target_position_.assign(n, 0.0);
  target_velocity_.assign(n, 0.0);
  target_effort_.assign(n, 0.0);
  target_kp_.assign(n, 0.0);
  target_kd_.assign(n, 0.0);

  has_command_ = false;

  command_sub_ = get_node()->create_subscription<ros_control_msgs::msg::MitJointCommand>(
      "~/commands", rclcpp::QoS(10),
      std::bind(&MitJointController::commandCallback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

void MitJointController::commandCallback(
    const ros_control_msgs::msg::MitJointCommand::SharedPtr msg)
{
  Command cmd;
  const std::size_t n = joint_names_.size();
  cmd.position.assign(n, 0.0);
  cmd.velocity.assign(n, 0.0);
  cmd.effort.assign(n, 0.0);
  cmd.kp.assign(n, 0.0);
  cmd.kd.assign(n, 0.0);

  const std::size_t count = msg->name.size();
  for (std::size_t i = 0; i < count; ++i) {
    const auto it = joint_index_.find(msg->name[i]);
    if (it == joint_index_.end()) {
      continue;
    }
    const std::size_t j = it->second;

    if (i < msg->position.size()) cmd.position[j] = msg->position[i];
    if (i < msg->velocity.size()) cmd.velocity[j] = msg->velocity[i];
    if (i < msg->effort.size()) cmd.effort[j] = msg->effort[i];
    if (i < msg->kp.size()) cmd.kp[j] = msg->kp[i];
    if (i < msg->kd.size()) cmd.kd[j] = msg->kd[i];
  }

  command_buffer_.writeFromNonRT(cmd);
}

controller_interface::CallbackReturn MitJointController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  has_command_ = false;
  std::fill(target_position_.begin(), target_position_.end(), 0.0);
  std::fill(target_velocity_.begin(), target_velocity_.end(), 0.0);
  std::fill(target_effort_.begin(), target_effort_.end(), 0.0);
  std::fill(target_kp_.begin(), target_kp_.end(), 0.0);
  std::fill(target_kd_.begin(), target_kd_.end(), 0.0);

  writeDamping();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitJointController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  writeDamping();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MitJointController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)period;

  auto command = command_buffer_.readFromRT();
  if (command != nullptr) {
    target_position_ = command->position;
    target_velocity_ = command->velocity;
    target_effort_ = command->effort;
    target_kp_ = command->kp;
    target_kd_ = command->kd;
    last_command_time_ = time;
    has_command_ = true;
  }

  const bool timed_out =
      !has_command_ ||
      (time - last_command_time_).seconds() > command_timeout_;

  const std::size_t n = joint_names_.size();
  for (std::size_t j = 0; j < n; ++j) {
    const auto state = state_interfaces_[j].get_optional();
    const double current_position = state.has_value() ? state.value() : 0.0;

    const double hold_position =
        has_command_ ? target_position_[j] : current_position;

    const double position = timed_out ? hold_position : target_position_[j];
    const double velocity = timed_out ? 0.0 : target_velocity_[j];
    const double effort = timed_out ? 0.0 : target_effort_[j];
    const double kp = timed_out ? 0.0 : target_kp_[j];
    const double kd = timed_out ? damping_kd_ : target_kd_[j];

    command_interfaces_[5 * j + 0].set_value(position);
    command_interfaces_[5 * j + 1].set_value(velocity);
    command_interfaces_[5 * j + 2].set_value(effort);
    command_interfaces_[5 * j + 3].set_value(kp);
    command_interfaces_[5 * j + 4].set_value(kd);
  }

  return controller_interface::return_type::OK;
}

void MitJointController::writeDamping()
{
  const std::size_t n = joint_names_.size();
  for (std::size_t j = 0; j < n; ++j) {
    const auto state = state_interfaces_[j].get_optional();
    const double current_position = state.has_value() ? state.value() : 0.0;

    command_interfaces_[5 * j + 0].set_value(current_position);
    command_interfaces_[5 * j + 1].set_value(0.0);
    command_interfaces_[5 * j + 2].set_value(0.0);
    command_interfaces_[5 * j + 3].set_value(0.0);
    command_interfaces_[5 * j + 4].set_value(damping_kd_);
  }
}

}  // namespace ros_control_controllers

PLUGINLIB_EXPORT_CLASS(
    ros_control_controllers::MitJointController,
    controller_interface::ControllerInterface)
