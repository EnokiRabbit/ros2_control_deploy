#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "ros_control_hardware/driver/joint/joint_model.hpp"
#include "ros_control_hardware/driver/protocol/motor_protocol.hpp"
#include "ros_control_hardware/driver/transport/transport.hpp"

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
  // State: position / velocity / effort
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> state_effort_;

  // Command: position / velocity / effort / kp / kd
  std::vector<double> cmd_position_;
  std::vector<double> cmd_velocity_;
  std::vector<double> cmd_effort_;
  std::vector<double> cmd_kp_;
  std::vector<double> cmd_kd_;

  // Driver layer: one transport per bus, one protocol per motor
  std::vector<std::unique_ptr<Transport>> transports_;
  std::vector<std::unique_ptr<MotorProtocol>> protocols_;
  std::vector<JointModel> joint_models_;

  // For each joint, index into transports_
  std::vector<std::size_t> joint_transport_;

  // Poll counter for protocols that need periodic state requests
  std::uint32_t cycle_count_{0};
};

}  // namespace ros_control_hardware
