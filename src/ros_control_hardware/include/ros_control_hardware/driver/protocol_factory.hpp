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

/// 单个电机的描述，全部来自 yaml
struct MotorSpec {
  std::string type;        ///< "encos" | "yiyou" | 以后新增
  uint8_t can_id{0};

  double direction{1.0};   ///< +1 / -1
  double gear_ratio{1.0};
  double offset{0.0};                  //todevice and tojoint

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

/// 协议注册表：新增电机类型只需注册一行，SystemHardware 不用改
class ProtocolFactory {
public:
  using Creator =
      std::function<std::unique_ptr<MotorProtocol>(Transport &, const MotorSpec &)>;

  static void registerType(const std::string & type, Creator creator);

  /// 找不到类型返回 nullptr
  static std::unique_ptr<MotorProtocol> create(
      const std::string & type, Transport & transport, const MotorSpec & spec);

  static std::vector<std::string> types();

private:
  static std::map<std::string, Creator> & registry();
};

/// 注册内置协议（encos / yiyou），在 on_init 或测试开头调用一次
void registerBuiltinProtocols();

}  // namespace ros_control_hardware
