#pragma once

#include <cstdint>
#include <string>

#include "ros_control_hardware/driver/protocol/motor_protocol.hpp"
#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// 意优 PP/PH 的型号参数
struct YiyouLimits {
  double rated_torque{90.0};     ///< 额定力矩，用于千分比换算
  double torque_limit{90.0};     ///< 力矩限幅
  double pulses_per_rev{65536.0};///< 编码器每圈脉冲数
};

class YiyouProtocol : public MotorProtocol {
public:
  YiyouProtocol(Transport & transport, uint8_t can_id, YiyouLimits limits = {});

  bool configure(std::string & error) override;
  void writeCommand(const DeviceCommand & command) override;
  void requestState() override;
  bool decode(const CanFrame & frame, DeviceState & state) override;
  [[nodiscard]] bool supportsMit() const override { return true; }

  // 协议常量：测试和调试时要用
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
  /// 组一帧 6 字节命令：CMD ADDR data32(大端)
  void sendCommand(uint8_t command, uint8_t address, int32_t value);

  Transport & transport_;
  uint8_t can_id_;
  YiyouLimits limits_;

  // 上次解码到的状态，PD 需要
  double position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
  bool has_position_{false};

  // 位置/速度交替轮询
  uint8_t poll_index_{0};
};

}  // namespace ros_control_hardware
