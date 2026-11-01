#pragma once

#include <deque>
#include <vector>

#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// In-process loopback transport: replaces a real CAN bus in tests.
class LoopbackTransport : public Transport {
public:
  bool open(std::string & error) override;
  void close() override;

  bool send(const CanFrame & frame) override;
  bool receive(CanFrame & frame) override;

  /// Inject a frame on the device side (simulates a motor reply on the bus).
  void push(const CanFrame & frame);

  /// Test helper: inspect the frames the host has sent.
  [[nodiscard]] const std::vector<CanFrame> & sentFrames() const { return sent_; }

private:
  std::deque<CanFrame> receive_queue_;  // device -> host
  std::vector<CanFrame> sent_;          // host -> device (recorded)
};

}  // namespace ros_control_hardware
