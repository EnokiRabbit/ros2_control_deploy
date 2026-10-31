#pragma once

#include <cstdint>
#include <string>

namespace ros_control_hardware {

/// 一帧原始总线数据（CAN 最多 8 字节）
struct CanFrame {
  uint32_t id{0};       // 标准帧 ID = 电机节点号
  uint8_t data[8]{};    // 数据区
  uint8_t len{0};       // 实际长度（0-8）
};

/// 传输层：只搬字节，不认识电机
class Transport {
public:
  virtual ~Transport() = default;

  virtual bool open(std::string & error) = 0;
  virtual void close() = 0;

  virtual bool send(const CanFrame & frame) = 0;

  /// 非阻塞：没有帧就返回 false，绝不等待
  virtual bool receive(CanFrame & frame) = 0;
};

}  // namespace ros_control_hardware
