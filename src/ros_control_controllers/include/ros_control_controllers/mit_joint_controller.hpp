#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "ros_control_msgs/msg/mit_joint_command.hpp"

namespace ros_control_controllers {

class MitJointController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  struct Command {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
    std::vector<double> kp;
    std::vector<double> kd;
  };

  void commandCallback(const ros_control_msgs::msg::MitJointCommand::SharedPtr msg);
  void writeDamping();

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_;

  double command_timeout_{0.5};
  double damping_kd_{0.0};

  rclcpp::Subscription<ros_control_msgs::msg::MitJointCommand>::SharedPtr command_sub_;
  realtime_tools::RealtimeBuffer<Command> command_buffer_;

  std::vector<double> target_position_;
  std::vector<double> target_velocity_;
  std::vector<double> target_effort_;
  std::vector<double> target_kp_;
  std::vector<double> target_kd_;

  bool has_command_{false};
  rclcpp::Time last_command_time_;
};

}  // namespace ros_control_controllers
