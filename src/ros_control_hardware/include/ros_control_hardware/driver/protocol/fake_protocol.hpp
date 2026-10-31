#pragma once

#include <string>

#include "ros_control_hardware/driver/transport/loopback_transport.hpp"
#include "ros_control_hardware/driver/protocol/motor_protocol.hpp"

namespace ros_control_hardware {

class FakeProtocol : public MotorProtocol {
public:
  FakeProtocol(LoopbackTransport & transport, uint32_t can_id, double inertia = 0.5);

  bool configure(std::string & error) override;
  void writeCommand(const DeviceCommand & command) override;
  void requestState() override;
  bool decode(const CanFrame & frame, DeviceState & state) override;

  [[nodiscard]] bool supportsMit() const override { return true; }

  void setTimeStep(double dt) { dt_ = dt; }
  [[nodiscard]] double position() const { return position_; }

private:
  void publishState();

  LoopbackTransport & transport_;
  uint32_t can_id_;
  double inertia_;
  double dt_{0.005};

  double position_{0.0};
  double velocity_{0.0};
  double effort_{0.0};
};

}  // namespace ros_control_hardware
