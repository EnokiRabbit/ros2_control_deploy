/**
 * @file fake_encos_motor.cpp
 * @brief Simulation-only ENCOS motor for a virtual CAN bus.
 *
 * Purpose
 * -------
 * Stands in for a real ENCOS motor when no hardware is available. It listens
 * on a CAN interface (usually vcan0), decodes the 8-byte MIT command frames
 * sent by the ros2_control hardware plugin, integrates a simple rigid-body
 * model, and replies with ENCOS type-1 state frames. This closes the
 * read/write loop so controllers and joint states can be tested end to end.
 *
 * It is a test tool: it is not part of the production plugin and is never
 * installed on the robot.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <thread>

#include "ros_control_hardware/driver/protocol/encos_protocol.hpp"
#include "ros_control_hardware/driver/transport/can_transport.hpp"

using namespace ros_control_hardware;

namespace {

/// State of one simulated motor, kept across frames so integration works.
struct Motor {
  EncosLimits limits;
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double inertia{0.5};
  double dt{0.01};
};

/// Process one command frame: decode, integrate, and send a state reply.
void step(Motor & motor, const CanFrame & frame, CanTransport & bus)
{
  const EncosLimits & limits = motor.limits;

  // Unpack the MIT command frame (inverse of EncosProtocol::writeCommand).
  const int kp_i = (static_cast<int>(frame.data[0]) << 7) |
                   ((static_cast<int>(frame.data[1]) >> 1) & 0x7F);
  const int kd_i = ((static_cast<int>(frame.data[1]) & 0x01) << 8) |
                   static_cast<int>(frame.data[2]);
  const int pos_i = (static_cast<int>(frame.data[3]) << 8) |
                    static_cast<int>(frame.data[4]);
  const int spd_i = (static_cast<int>(frame.data[5]) << 4) |
                    (static_cast<int>(frame.data[6]) >> 4);
  const int tor_i = ((static_cast<int>(frame.data[6]) & 0x0F) << 8) |
                    static_cast<int>(frame.data[7]);

  // Dequantize back to physical units.
  const double kp =
      EncosProtocol::uintToFloat(kp_i, limits.kp_min, limits.kp_max, 12);
  const double kd =
      EncosProtocol::uintToFloat(kd_i, limits.kd_min, limits.kd_max, 9);
  const double target =
      EncosProtocol::uintToFloat(pos_i, limits.pos_min, limits.pos_max, 16);
  const double target_velocity =
      EncosProtocol::uintToFloat(spd_i, limits.spd_min, limits.spd_max, 12);
  const double feed_forward =
      EncosProtocol::uintToFloat(tor_i, limits.torque_min, limits.torque_max, 12);

  if (std::getenv("FAKE_DEBUG") != nullptr) {
    std::printf("id=%u kp=%.3f kd=%.3f target=%.3f q=%.3f dq=%.3f ff=%.3f\n",
                frame.id, kp, kd, target, motor.position, motor.velocity, feed_forward);
  }

  // PD + feed-forward, then a simple rigid-body integration.
  double torque = kp * (target - motor.position) +
                  kd * (target_velocity - motor.velocity) +
                  feed_forward;
  torque = std::clamp(torque, limits.torque_min, limits.torque_max);

  // Quantization maps zero torque to a small non-zero value. Without friction
  // the simulated motor would drift forever, so mimic static friction here.
  if (std::fabs(torque) < 0.05) {
    torque = 0.0;
  }

  motor.effort = torque;
  motor.velocity += torque / motor.inertia * motor.dt;
  motor.position += motor.velocity * motor.dt;

  // Keep the state encodable (mirrors encoder saturation on real hardware).
  motor.position = std::clamp(motor.position, limits.pos_min, limits.pos_max);
  motor.velocity = std::clamp(motor.velocity, limits.spd_min, limits.spd_max);

  // Encode a type-1 state reply.
  CanFrame reply;
  reply.id = frame.id;
  reply.len = 8;

  const int out_pos = EncosProtocol::floatToUint(
      motor.position, limits.pos_min, limits.pos_max, 16);
  const int out_spd = EncosProtocol::floatToUint(
      motor.velocity, limits.spd_min, limits.spd_max, 12);
  const double current =
      (std::fabs(limits.kt) > 1.0e-6) ? motor.effort / limits.kt : 0.0;
  const int out_cur = EncosProtocol::floatToUint(
      current, limits.current_min, limits.current_max, 12);

  reply.data[0] = 0;  // error code
  reply.data[1] = static_cast<std::uint8_t>(out_pos >> 8);
  reply.data[2] = static_cast<std::uint8_t>(out_pos & 0xFF);
  reply.data[3] = static_cast<std::uint8_t>(out_spd >> 4);
  reply.data[4] = static_cast<std::uint8_t>(((out_spd & 0x0F) << 4) |
                                            ((out_cur >> 8) & 0x0F));
  reply.data[5] = static_cast<std::uint8_t>(out_cur & 0xFF);
  reply.data[6] = 50;  // 0 C
  reply.data[7] = 50;  // 0 C

  bus.send(reply);
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string interface_name = (argc > 1) ? argv[1] : "vcan0";

  CanTransport bus(interface_name);
  std::string error;
  if (!bus.open(error)) {
    std::printf("open failed: %s\n", error.c_str());
    return 1;
  }

  // Simulated motors, keyed by CAN id.
  std::map<std::uint8_t, Motor> motors;
  for (std::uint8_t id = 1; id <= 14; ++id) {
    motors[id];
  }

  std::printf("fake ENCOS motor on %s, ids 1..14\n", interface_name.c_str());

  // Main loop: drain the bus, dispatch each command frame to its motor.
  while (true) {
    CanFrame frame;
    while (bus.receive(frame)) {
      const auto it = motors.find(static_cast<std::uint8_t>(frame.id));
      if (it == motors.end() || frame.len != 8) {
        continue;
      }
      step(it->second, frame, bus);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
