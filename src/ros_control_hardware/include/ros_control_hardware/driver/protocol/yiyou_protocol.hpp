#pragma once

#include <cstdint>
#include <string>

#include "ros_control_hardware/driver/protocol/motor_protocol.hpp"
#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// Model parameters for Yiyou PP/PH motors.
struct YiyouLimits {
  double rated_torque{90.0};     ///< Rated torque, used for per-mille conversion
  double torque_limit{90.0};     ///< Torque limit
  double pulses_per_rev{65536.0};///< Encoder pulses per revolution
};

class YiyouProtocol : public MotorProtocol {
public:
  YiyouProtocol(Transport & transport, uint8_t can_id, YiyouLimits limits = {});

  bool configure(std::string & error) override;
  void writeCommand(const DeviceCommand & command) override;
  void requestState() override;
  bool decode(const CanFrame & frame, DeviceState & state) override;
  [[nodiscard]] bool supportsMit() const override { return true; }

  // Protocol constants used by tests and debugging
  static constexpr uint8_t kCmdWrite = 0x01;
  static constexpr uint8_t kCmdRead = 0x03;
  static constexpr uint8_t kCmdReadReply = 0x04;
  static constexpr uint8_t kCmdFastWrite = 0x05;

  static constexpr uint8_t kRegCurrentTorque = 0x05;
  static constexpr uint8_t kRegCurrentSpeed = 0x06;
  static constexpr uint8_t kRegCurrentPosition = 0x07;
  static constexpr uint8_t kRegTargetTorque = 0x08;
  static constexpr uint8_t kRegWorkMode = 0x0F;
  static constexpr uint8_t kRegEnableState = 0x10;

private:
  /// Build one 6-byte command frame: CMD ADDR data32 (big-endian).
  void sendCommand(uint8_t command, uint8_t address, int32_t value);

  Transport & transport_;
  uint8_t can_id_;
  YiyouLimits limits_;

  // Last decoded state, required by the PD law
  double position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
  bool has_position_{false};

  // Alternate polling between position and velocity
  uint8_t poll_index_{0};
};

}  // namespace ros_control_hardware
