#pragma once

#include <cstdint>
#include <string>

#include "ros_control_hardware/driver/protocol/motor_protocol.hpp"
#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// ENCOS EC 系列的量化量程（按型号不同）
struct EncosLimits {
  double kp_min{0.0};
  double kp_max{500.0};
  double kd_min{0.0};
  double kd_max{5.0};
  double pos_min{-12.5};
  double pos_max{12.5};
  double spd_min{-18.0};
  double spd_max{18.0};
  double torque_min{-90.0};
  double torque_max{90.0};
  double current_min{-60.0};
  double current_max{60.0};
  double kt{2.1};   // 电流 -> 力矩的系数
};

class EncosProtocol : public MotorProtocol {
public:
  EncosProtocol(Transport & transport, uint8_t can_id, EncosLimits limits = {});

  bool configure(std::string & error) override;
  void writeCommand(const DeviceCommand & command) override;
  void requestState() override;
  bool decode(const CanFrame & frame, DeviceState & state) override;
  [[nodiscard]] bool supportsMit() const override { return true; }

  // 测试用：暴露量化/反量化
  static int floatToUint(double value, double min, double max, int bits);
  static double uintToFloat(int value, double min, double max, int bits);

private:
  Transport & transport_;
  uint8_t can_id_;
  EncosLimits limits_;
};

}  // namespace ros_control_hardware
