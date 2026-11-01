#include "ros_control_hardware/driver/protocol/encos_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace ros_control_hardware {

EncosProtocol::EncosProtocol(Transport & transport, uint8_t can_id, EncosLimits limits)
    : transport_{transport}, can_id_{can_id}, limits_{limits}
{
}

int EncosProtocol::floatToUint(double value, double min, double max, int bits)
{
  const double clamped = std::clamp(value, min, max);
  const double span = max - min;
  return static_cast<int>((clamped - min) * static_cast<double>((1 << bits) - 1) / span);
}

double EncosProtocol::uintToFloat(int value, double min, double max, int bits)
{
  const double span = max - min;
  return static_cast<double>(value) * span / static_cast<double>((1 << bits) - 1) + min;
}

bool EncosProtocol::configure(std::string & error)
{
  (void)error;
  // ENCOS 的 MIT 命令本身就会使能电机，不需要额外初始化帧
  return true;
}

void EncosProtocol::writeCommand(const DeviceCommand & command)
{
  const double kp = std::clamp(command.kp, limits_.kp_min, limits_.kp_max);
  const double kd = std::clamp(command.kd, limits_.kd_min, limits_.kd_max);
  const double pos = std::clamp(command.position, limits_.pos_min, limits_.pos_max);
  const double spd = std::clamp(command.velocity, limits_.spd_min, limits_.spd_max);
  const double torque = std::clamp(command.effort, limits_.torque_min, limits_.torque_max);

  const int kp_i = floatToUint(kp, limits_.kp_min, limits_.kp_max, 12);
  const int kd_i = floatToUint(kd, limits_.kd_min, limits_.kd_max, 9);
  const int pos_i = floatToUint(pos, limits_.pos_min, limits_.pos_max, 16);
  const int spd_i = floatToUint(spd, limits_.spd_min, limits_.spd_max, 12);
  const int torque_i = floatToUint(torque, limits_.torque_min, limits_.torque_max, 12);

  CanFrame frame;
  frame.id = can_id_;
  frame.len = 8;
  frame.data[0] = static_cast<uint8_t>(kp_i >> 7);
  frame.data[1] = static_cast<uint8_t>(((kp_i & 0x7F) << 1) | ((kd_i >> 8) & 0x01));
  frame.data[2] = static_cast<uint8_t>(kd_i & 0xFF);
  frame.data[3] = static_cast<uint8_t>(pos_i >> 8);
  frame.data[4] = static_cast<uint8_t>(pos_i & 0xFF);
  frame.data[5] = static_cast<uint8_t>(spd_i >> 4);
  frame.data[6] = static_cast<uint8_t>(((spd_i & 0x0F) << 4) | ((torque_i >> 8) & 0x0F));
  frame.data[7] = static_cast<uint8_t>(torque_i & 0xFF);

  transport_.send(frame);
}

void EncosProtocol::requestState()
{
  // ENCOS 收到 MIT 命令后会主动回 type-1 帧，无需轮询
}

bool EncosProtocol::decode(const CanFrame & frame, DeviceState & state)
{
  if (frame.id != can_id_ || frame.len < 8) {
    return false;
  }

  const uint8_t error = frame.data[0] & 0x1F;
  const int pos_i = (frame.data[1] << 8) | frame.data[2];
  const int spd_i = (frame.data[3] << 4) | (frame.data[4] >> 4);
  const int current_i = ((frame.data[4] & 0x0F) << 8) | frame.data[5];

  const double position = uintToFloat(pos_i, limits_.pos_min, limits_.pos_max, 16);
  const double velocity = uintToFloat(spd_i, limits_.spd_min, limits_.spd_max, 12);
  const double current = uintToFloat(current_i, limits_.current_min, limits_.current_max, 12);

  state.position = position;
  state.velocity = velocity;
  state.effort = current * limits_.kt;
  state.temperature = (static_cast<double>(frame.data[6]) - 50.0) / 2.0;
  state.valid = true;
  (void)error;   // 错误码暂时只做诊断，不影响有效性
  return true;
}

}  // namespace ros_control_hardware
