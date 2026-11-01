#include "ros_control_hardware/system_hardware.hpp"

#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace ros_control_hardware {

hardware_interface::CallbackReturn SystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
  // Must call the base class: it stores info_ and validates interface declarations
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();

  // State buffers: sized once and never resized afterwards
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  state_effort_.assign(n, 0.0);

  // Command buffers: NaN means "no valid command yet"
  const double nan = std::numeric_limits<double>::quiet_NaN();
  cmd_position_.assign(n, nan);
  cmd_velocity_.assign(n, nan);
  cmd_effort_.assign(n, nan);
  cmd_kp_.assign(n, nan);
  cmd_kd_.assign(n, nan);

  RCLCPP_INFO(rclcpp::get_logger("ros_control_hardware"),
              "SystemHardware: %zu joints", n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 5);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &cmd_effort_[i]);
    interfaces.emplace_back(name, "kp", &cmd_kp_[i]);
    interfaces.emplace_back(name, "kd", &cmd_kd_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn SystemHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // Step 2.2: open transports and build protocols here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // Step 2.2: close transports and release protocols here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // Step 2.2: clear queues and reset commands to NaN here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // Step 2.3: send one damping command here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // Step 2.2: receive from Transport -> decode with Protocol -> write state_position_ etc.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // Step 2.3: read cmd_* -> build frames -> send through Transport
  return hardware_interface::return_type::OK;
}

}  // namespace ros_control_hardware

// Register this class as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(ros_control_hardware::SystemHardware,
                       hardware_interface::SystemInterface)
