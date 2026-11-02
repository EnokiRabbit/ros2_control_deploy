#include "ros_control_hardware/system_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "ros_control_hardware/driver/protocol_factory.hpp"
#include "ros_control_hardware/driver/transport/loopback_transport.hpp"

namespace ros_control_hardware {
namespace {

// Read a string parameter from a joint, falling back to a default value.
std::string jointParam(
    const hardware_interface::ComponentInfo & joint,
    const std::string & key,
    const std::string & fallback)
{
  const auto it = joint.parameters.find(key);
  return (it == joint.parameters.end()) ? fallback : it->second;
}

// Read a numeric parameter from a joint, falling back to a default value.
double jointParamDouble(
    const hardware_interface::ComponentInfo & joint,
    const std::string & key,
    double fallback)
{
  const std::string value = jointParam(joint, key, "");
  return value.empty() ? fallback : std::stod(value);
}

}  // namespace

hardware_interface::CallbackReturn SystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
  // Must call the base class: it stores info_ and validates interface declarations
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = info_.joints.size();

  // State buffers: sized once and never resized afterwards
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  state_effort_.assign(n, 0.0);

  // Command buffers: NaN means "no valid command yet"
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  cmd_position_.assign(n, kNaN);
  cmd_velocity_.assign(n, kNaN);
  cmd_effort_.assign(n, kNaN);
  cmd_kp_.assign(n, kNaN);
  cmd_kd_.assign(n, kNaN);

  // Reset driver-layer state (on_init may run again after a clean restart)
  transports_.clear();
  protocols_.clear();
  joint_models_.clear();
  joint_transport_.clear();

  registerBuiltinProtocols();

  std::map<std::string, std::size_t> bus_index;

  for (std::size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];

    try {
      // Resolve the bus, creating a transport on first use
      const std::string bus = jointParam(joint, "bus", "loopback0");

      auto it = bus_index.find(bus);
      if (it == bus_index.end()) {
        const std::size_t index = transports_.size();
        // TODO(2.6): replace with CanTransport(bus)
        transports_.push_back(std::make_unique<LoopbackTransport>());
        it = bus_index.emplace(bus, index).first;
      }

      // Build the motor description from joint parameters
      MotorSpec spec;
      spec.type = jointParam(joint, "motor_type", "encos");
      spec.can_id = static_cast<std::uint8_t>(
          std::stoi(jointParam(joint, "can_id", "1")));
      spec.direction = jointParamDouble(joint, "direction", 1.0);
      spec.gear_ratio = jointParamDouble(joint, "gear_ratio", 1.0);
      spec.offset = jointParamDouble(joint, "offset", 0.0);
      spec.rated_torque = jointParamDouble(joint, "rated_torque", 90.0);
      spec.torque_limit = jointParamDouble(joint, "torque_limit", 90.0);

      auto protocol =
          ProtocolFactory::create(spec.type, *transports_[it->second], spec);

      if (!protocol) {
        RCLCPP_ERROR(rclcpp::get_logger("ros_control_hardware"),
                     "unknown motor_type '%s' for joint '%s'",
                     spec.type.c_str(), joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      protocols_.push_back(std::move(protocol));
      joint_models_.emplace_back(spec.direction, spec.gear_ratio, spec.offset);
      joint_transport_.push_back(it->second);

      RCLCPP_INFO(rclcpp::get_logger("ros_control_hardware"),
                  "joint %s: bus=%s type=%s id=%u",
                  joint.name.c_str(), bus.c_str(), spec.type.c_str(),
                  static_cast<unsigned>(spec.can_id));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("ros_control_hardware"),
                   "joint %s: invalid parameter: %s",
                   joint.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ros_control_hardware"),
              "SystemHardware: %zu joints, %zu buses",
              n, transports_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &state_position_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &state_velocity_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT,
                            &state_effort_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 5);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &cmd_position_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &cmd_velocity_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT,
                            &cmd_effort_[i]);
    interfaces.emplace_back(name, "kp", &cmd_kp_[i]);
    interfaces.emplace_back(name, "kd", &cmd_kd_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn SystemHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  for (auto & transport : transports_) {
    std::string error;
    if (!transport->open(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("ros_control_hardware"),
                   "failed to open transport: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (auto & protocol : protocols_) {
    std::string error;
    if (!protocol->configure(error)) {
      RCLCPP_ERROR(rclcpp::get_logger("ros_control_hardware"),
                   "failed to configure protocol: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  for (auto & transport : transports_) {
    transport->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  // Drop stale commands so the first write holds the current position
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  std::fill(cmd_position_.begin(), cmd_position_.end(), kNaN);
  std::fill(cmd_velocity_.begin(), cmd_velocity_.end(), kNaN);
  std::fill(cmd_effort_.begin(), cmd_effort_.end(), kNaN);
  std::fill(cmd_kp_.begin(), cmd_kp_.end(), kNaN);
  std::fill(cmd_kd_.begin(), cmd_kd_.end(), kNaN);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  // Safety fallback: zero stiffness, light damping
  for (std::size_t j = 0; j < protocols_.size(); ++j) {
    DeviceCommand command;
    command.position = joint_models_[j].toDevicePosition(state_position_[j]);
    command.kp = 0.0;
    command.kd = 1.0;
    protocols_[j]->writeCommand(command);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  for (std::size_t t = 0; t < transports_.size(); ++t) {
    CanFrame frame;
    while (transports_[t]->receive(frame)) {
      for (std::size_t j = 0; j < protocols_.size(); ++j) {
        if (joint_transport_[j] != t) {
          continue;
        }

        DeviceState state;
        if (!protocols_[j]->decode(frame, state) || !state.valid) {
          continue;
        }

        state_position_[j] = joint_models_[j].toJointPosition(state.position);
        state_velocity_[j] = joint_models_[j].toJointVelocity(state.velocity);
        state_effort_[j] = joint_models_[j].toJointEffort(state.effort);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  ++cycle_count_;
  const bool poll = (cycle_count_ % 10 == 0);   // ~20 Hz polling

  for (std::size_t j = 0; j < protocols_.size(); ++j) {
    // Hold the current position until the controller sends a valid command
    const double target_position =
        std::isnan(cmd_position_[j]) ? state_position_[j] : cmd_position_[j];

    DeviceCommand command;
    command.position = joint_models_[j].toDevicePosition(target_position);
    command.velocity = joint_models_[j].toDeviceVelocity(
        std::isnan(cmd_velocity_[j]) ? 0.0 : cmd_velocity_[j]);
    command.effort = joint_models_[j].toDeviceEffort(
        std::isnan(cmd_effort_[j]) ? 0.0 : cmd_effort_[j]);
    command.kp = std::isnan(cmd_kp_[j]) ? 0.0 : cmd_kp_[j];
    command.kd = std::isnan(cmd_kd_[j]) ? 0.0 : cmd_kd_[j];

    protocols_[j]->writeCommand(command);

    if (poll) {
      protocols_[j]->requestState();
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ros_control_hardware

// Register this class as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(ros_control_hardware::SystemHardware,
                       hardware_interface::SystemInterface)
