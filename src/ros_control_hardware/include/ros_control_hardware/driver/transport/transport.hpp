#pragma once

#include <cstdint>
#include <string>

namespace ros_control_hardware {

/// One raw bus frame (CAN payload is at most 8 bytes).
struct CanFrame {
  uint32_t id{0};       // Standard frame ID = motor node id
  uint8_t data[8]{};    // Payload
  uint8_t len{0};       // Actual length (0-8)
};

/// Transport layer: moves bytes only, knows nothing about motors.
class Transport {
public:
  virtual ~Transport() = default;

  virtual bool open(std::string & error) = 0;
  virtual void close() = 0;

  virtual bool send(const CanFrame & frame) = 0;

  /// Non-blocking: returns false when no frame is available, never waits.
  virtual bool receive(CanFrame & frame) = 0;
};

}  // namespace ros_control_hardware
