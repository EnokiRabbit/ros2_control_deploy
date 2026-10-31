#include "ros_control_hardware/system_hardware.hpp"

#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace ros_control_hardware {

hardware_interface::CallbackReturn SystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
  // 必须调用基类：它会保存 info_、检查接口声明
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();

  // 状态缓冲：一次定长，之后绝不再 resize
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  state_effort_.assign(n, 0.0);

  // 命令缓冲：NaN = "还没有有效命令"
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
  // 2.2 在这里打开 Transport、建 Protocol
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // 2.2 在这里关闭 Transport、释放 Protocol
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // 2.2 在这里清队列、把命令重置为 NaN
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // 2.3 在这里发一次阻尼
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // 2.2 在这里从 Transport 收帧 -> Protocol 解码 -> 写 state_position_ 等
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // 2.3 在这里读 cmd_* -> 组帧 -> Transport 发送
  return hardware_interface::return_type::OK;
}

}  // namespace ros_control_hardware

// 把这个类注册成 pluginlib 插件
PLUGINLIB_EXPORT_CLASS(ros_control_hardware::SystemHardware,
                       hardware_interface::SystemInterface)
