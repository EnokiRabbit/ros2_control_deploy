#include "ros_control_hardware/driver/transport/loopback_transport.hpp"

namespace ros_control_hardware {

bool LoopbackTransport::open(std::string & error)
{
  (void)error;
  return true;
}

void LoopbackTransport::close()
{
  receive_queue_.clear();
  sent_.clear();
}

bool LoopbackTransport::send(const CanFrame & frame)
{
  // Bound the history so long-running loops do not grow memory forever.
  constexpr std::size_t kMaxRecordedFrames = 10000;
  if (sent_.size() >= kMaxRecordedFrames) {
    sent_.clear();
  }
  sent_.push_back(frame);
  return true;
}

bool LoopbackTransport::receive(CanFrame & frame)
{
  if (receive_queue_.empty()) {
    return false;
  }
  frame = receive_queue_.front();
  receive_queue_.pop_front();
  return true;
}

void LoopbackTransport::push(const CanFrame & frame)
{
  receive_queue_.push_back(frame);
}

}  // namespace ros_control_hardware
