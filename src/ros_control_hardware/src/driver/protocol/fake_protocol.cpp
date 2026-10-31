#include "ros_control_hardware/driver/protocol/fake_protocol.hpp"

#include <cstring>

namespace ros_control_hardware {

FakeProtocol::FakeProtocol(LoopbackTransport & transport, uint32_t can_id, double inertia)
    : transport_{transport}, can_id_{can_id}, inertia_{inertia}
{
}

bool FakeProtocol::configure(std::string & error)
{
  (void)error;
  return true;
}

void FakeProtocol::writeCommand(const DeviceCommand & command)
{
  // 1. PD：算需要的力矩
  effort_ = command.kp * (command.position - position_) +
            command.kd * (command.velocity - velocity_) +
            command.effort;

  // 2. 牛二定律：a = F / m，再积分
  velocity_ += effort_ / inertia_ * dt_;
  position_ += velocity_ * dt_;

  // 3. 把新状态作为回包发回主机
  publishState();
}

void FakeProtocol::requestState()
{
  publishState();
}

void FakeProtocol::publishState()
{
  CanFrame frame;
  frame.id = can_id_;
  frame.len = 8;

  const auto position = static_cast<float>(position_);
  const auto velocity = static_cast<float>(velocity_);
  std::memcpy(frame.data, &position, sizeof(position));
  std::memcpy(frame.data + 4, &velocity, sizeof(velocity));

  transport_.push(frame);
}

bool FakeProtocol::decode(const CanFrame & frame, DeviceState & state)
{
  if (frame.id != can_id_ || frame.len < 8) {
    return false;
  }

  float position = 0.0F;
  float velocity = 0.0F;
  std::memcpy(&position, frame.data, sizeof(position));
  std::memcpy(&velocity, frame.data + 4, sizeof(velocity));

  state.position = position;
  state.velocity = velocity;
  state.effort = effort_;
  state.valid = true;
  return true;
}

}  // namespace ros_control_hardware
