#pragma once

#include <string>

#include "ros_control_hardware/driver/transport/transport.hpp"

namespace ros_control_hardware {

/// SocketCAN transport: reads and writes a Linux CAN interface (non-blocking).
class CanTransport : public Transport {
public:
  explicit CanTransport(std::string interface_name);

  bool open(std::string & error) override;
  void close() override;
  bool send(const CanFrame & frame) override;
  bool receive(CanFrame & frame) override;

private:
  std::string interface_;
  int socket_{-1};
};

}  // namespace ros_control_hardware
