#pragma once

#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace ros_control_hardware {

class SystemHardware : public hardware_interface::SystemInterface {
public:
  // ---- ros2_control lifecycle ----
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  // ---- runtime cycle ----
  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // state：pos / vel / eff
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> state_effort_;

  // command：pos / vel / eff / kp / kd
  std::vector<double> cmd_position_;
  std::vector<double> cmd_velocity_;
  std::vector<double> cmd_effort_;
  std::vector<double> cmd_kp_;
  std::vector<double> cmd_kd_;
};

}  // namespace ros_control_hardware
