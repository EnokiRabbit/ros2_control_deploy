#include "ros_control_hardware/driver/protocol_factory.hpp"

#include <utility>

#include "ros_control_hardware/driver/protocol/encos_protocol.hpp"
#include "ros_control_hardware/driver/protocol/yiyou_protocol.hpp"

namespace ros_control_hardware {

std::map<std::string, ProtocolFactory::Creator> & ProtocolFactory::registry()
{
  // Function-local static: constructed on first use, thread-safe since C++11
  static std::map<std::string, Creator> instance;
  return instance;
}

void ProtocolFactory::registerType(const std::string & type, Creator creator)
{
  registry()[type] = std::move(creator);
}

std::unique_ptr<MotorProtocol> ProtocolFactory::create(
    const std::string & type, Transport & transport, const MotorSpec & spec)
{
  const auto it = registry().find(type);
  if (it == registry().end()) {
    return nullptr;
  }
  return it->second(transport, spec);
}

std::vector<std::string> ProtocolFactory::types()
{
  std::vector<std::string> names;
  names.reserve(registry().size());
  for (const auto & item : registry()) {
    names.push_back(item.first);
  }
  return names;
}

void registerBuiltinProtocols()
{
  ProtocolFactory::registerType(
      "encos",
      [](Transport & transport, const MotorSpec & spec) -> std::unique_ptr<MotorProtocol> {
        EncosLimits limits;
        limits.kp_max = spec.kp_max;
        limits.kd_max = spec.kd_max;
        limits.pos_min = spec.position_min;
        limits.pos_max = spec.position_max;
        limits.spd_min = spec.velocity_min;
        limits.spd_max = spec.velocity_max;
        limits.torque_min = -spec.torque_limit;
        limits.torque_max = spec.torque_limit;
        limits.current_min = spec.current_min;
        limits.current_max = spec.current_max;
        limits.kt = spec.kt;
        return std::make_unique<EncosProtocol>(transport, spec.can_id, limits);
      });

  ProtocolFactory::registerType(
      "yiyou",
      [](Transport & transport, const MotorSpec & spec) -> std::unique_ptr<MotorProtocol> {
        YiyouLimits limits;
        limits.rated_torque = spec.rated_torque;
        limits.torque_limit = spec.torque_limit;
        return std::make_unique<YiyouProtocol>(transport, spec.can_id, limits);
      });
}

}  // namespace ros_control_hardware
