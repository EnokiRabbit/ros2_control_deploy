#pragma once

#include <string>

#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// Device-side intent (device units; direction and gear ratio are handled by JointModel).
struct DeviceCommand {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};   // Feed-forward torque
  double kp{0.0};
  double kd{0.0};
};

/// Device-side feedback (device units).
struct DeviceState {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double temperature{0.0};
  bool valid{false};
};

/// Protocol layer: encodes intent into frames and decodes replies into state.
class MotorProtocol {
public:
  virtual ~MotorProtocol() = default;

  /// Power-up initialization: set mode, enable, read state once.
  virtual bool configure(std::string & error) = 0;

  /// Encode and send one control command.
  virtual void writeCommand(const DeviceCommand & command) = 0;

  /// Actively poll state (some motors require it).
  virtual void requestState() = 0;

  /// Decode one frame; fills state and returns true if the frame belongs to this device.
  virtual bool decode(const CanFrame & frame, DeviceState & state) = 0;

  /// Whether this motor natively supports MIT control.
  [[nodiscard]] virtual bool supportsMit() const = 0;
};

}  // namespace ros_control_hardware
