#include "ros_control_hardware/driver/protocol/yiyou_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace ros_control_hardware {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

int32_t decodeInt32BigEndian(const uint8_t * data)
{
  const uint32_t raw = (static_cast<uint32_t>(data[2]) << 24) |
                       (static_cast<uint32_t>(data[3]) << 16) |
                       (static_cast<uint32_t>(data[4]) << 8) |
                       static_cast<uint32_t>(data[5]);
  return static_cast<int32_t>(raw);
}

}  // namespace

YiyouProtocol::YiyouProtocol(Transport & transport, uint8_t can_id, YiyouLimits limits)
    : transport_{transport}, can_id_{can_id}, limits_{limits}
{
}

bool YiyouProtocol::configure(std::string & error)
{
  (void)error;

  // 切到电流（力矩）模式
  sendCommand(kCmdWrite, kRegWorkMode, 4);
  // 使能
  sendCommand(kCmdWrite, kRegEnableState, 1);
  // 读一次当前位置，初始化 position_
  sendCommand(kCmdRead, kRegCurrentPosition, 0);
  return true;
}

void YiyouProtocol::writeCommand(const DeviceCommand & command)
{
  // 意优没有 MIT，主机自己算 PD
  const double torque = command.kp * (command.position - position_) +
                        command.kd * (command.velocity - velocity_) +
                        command.effort;

  torque_ = std::clamp(torque, -limits_.torque_limit, limits_.torque_limit);

  const double ratio =
      (limits_.rated_torque > 1.0e-4) ? (torque_ / limits_.rated_torque) : 0.0;
  const auto per_mille = static_cast<int32_t>(std::lround(ratio * 1000.0));

  sendCommand(kCmdFastWrite, kRegTargetTorque, per_mille);
}

void YiyouProtocol::requestState()
{
  // 一帧只能读一个寄存器，位置和速度交替读
  if (poll_index_ == 0) {
    sendCommand(kCmdRead, kRegCurrentPosition, 0);
    poll_index_ = 1;
  } else {
    sendCommand(kCmdRead, kRegCurrentSpeed, 0);
    poll_index_ = 0;
  }
}

bool YiyouProtocol::decode(const CanFrame & frame, DeviceState & state)
{
  if (frame.id != can_id_ || frame.len < 6 || frame.data[0] != kCmdReadReply) {
    return false;
  }

  const int32_t value = decodeInt32BigEndian(frame.data);

  switch (frame.data[1]) {
    case kRegCurrentPosition:
      position_ = static_cast<double>(value) / limits_.pulses_per_rev * kTwoPi;
      has_position_ = true;
      break;

    case kRegCurrentSpeed:
      velocity_ = static_cast<double>(value) / limits_.pulses_per_rev * kTwoPi;
      break;

    case kRegCurrentTorque:
      torque_ = static_cast<double>(value) / 1000.0 * limits_.rated_torque;
      break;

    default:
      return false;
  }

  state.position = position_;
  state.velocity = velocity_;
  state.effort = torque_;
  state.valid = has_position_;
  return true;
}

void YiyouProtocol::sendCommand(uint8_t command, uint8_t address, int32_t value)
{
  CanFrame frame;
  frame.id = can_id_;
  frame.len = 6;

  const uint32_t raw = static_cast<uint32_t>(value);
  frame.data[0] = command;
  frame.data[1] = address;
  frame.data[2] = static_cast<uint8_t>((raw >> 24) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  frame.data[4] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  frame.data[5] = static_cast<uint8_t>(raw & 0xFF);

  transport_.send(frame);
}

}  // namespace ros_control_hardware
