#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ros_control_hardware/driver/protocol/motor_protocol.hpp"
#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// Description of a single motor, fully populated from yaml.
struct MotorSpec {
  std::string type;        ///< "encos" | "yiyou" | future types
  uint8_t can_id{0};

  double direction{1.0};   ///< +1 / -1
  double gear_ratio{1.0};
  double offset{0.0};      // Used by toDevice / toJoint

  double rated_torque{90.0};
  double torque_limit{90.0};

  double kp_max{500.0};
  double kd_max{5.0};
  double position_min{-12.5};
  double position_max{12.5};
  double velocity_min{-18.0};
  double velocity_max{18.0};
  double current_min{-60.0};
  double current_max{60.0};
  double kt{2.1};
};

/// Protocol registry: adding a motor type requires one registration line only.
class ProtocolFactory {
public:
  using Creator =
      std::function<std::unique_ptr<MotorProtocol>(Transport &, const MotorSpec &)>;

  static void registerType(const std::string & type, Creator creator);

  /// Returns nullptr when the type is unknown.
  static std::unique_ptr<MotorProtocol> create(
      const std::string & type, Transport & transport, const MotorSpec & spec);

  static std::vector<std::string> types();

private:
  static std::map<std::string, Creator> & registry();
};

/// Register built-in protocols (encos / yiyou); call once in on_init or at test start.
void registerBuiltinProtocols();

}  // namespace ros_control_hardware
