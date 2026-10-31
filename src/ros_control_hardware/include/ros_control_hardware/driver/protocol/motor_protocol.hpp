#pragma once

#include <string>

#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// 设备侧意图（设备单位；方向/减速比在 JointModel 处理）
struct DeviceCommand {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};   // 前馈力矩
  double kp{0.0};
  double kd{0.0};
};

/// 设备侧反馈（设备单位）
struct DeviceState {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double temperature{0.0};
  bool valid{false};
};

/// 协议层：把意图编码成帧，把回包解码成状态
class MotorProtocol {
public:
  virtual ~MotorProtocol() = default;

  /// 上电初始化：切模式、使能、读一次状态
  virtual bool configure(std::string & error) = 0;

  /// 编码并发送一条控制命令
  virtual void writeCommand(const DeviceCommand & command) = 0;

  /// 主动查询状态（有的电机要轮询）
  virtual void requestState() = 0;

  /// 解一帧；是本设备的帧就填 state 并返回 true
  virtual bool decode(const CanFrame & frame, DeviceState & state) = 0;

  /// 这款电机是否原生支持 MIT
  [[nodiscard]] virtual bool supportsMit() const = 0;
};

}  // namespace ros_control_hardware
