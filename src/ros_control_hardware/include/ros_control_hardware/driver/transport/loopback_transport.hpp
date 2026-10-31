#pragma once

#include <deque>
#include <vector>

#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// 进程内回环：测试时替代真实 CAN
class LoopbackTransport : public Transport {
public:
  bool open(std::string & error) override;
  void close() override;

  bool send(const CanFrame & frame) override;
  bool receive(CanFrame & frame) override;

  /// 设备侧注入回包（模拟总线收到电机反馈）
  void push(const CanFrame & frame);

  /// 测试用：查看主机发出了哪些帧
  [[nodiscard]] const std::vector<CanFrame> & sentFrames() const { return sent_; }

private:
  std::deque<CanFrame> receive_queue_;  // 设备 -> 主机
  std::vector<CanFrame> sent_;          // 主机 -> 设备（留档）
};

}  // namespace ros_control_hardware
