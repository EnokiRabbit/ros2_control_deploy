// 驱动层验收：不需要 CAN，也不需要真电机。
//
// 覆盖：
//   1. JointModel 的往返换算
//   2. FakeProtocol 在 PD 控制下能否收敛到目标
//   3. EncosProtocol 的 8 字节编码 / type-1 解码
//   4. YiyouProtocol 的力矩千分比编码 / READ_REPLY 解码

#include <cmath>
#include <cstdio>

#include "ros_control_hardware/driver/joint/joint_model.hpp"
#include "ros_control_hardware/driver/protocol/encos_protocol.hpp"
#include "ros_control_hardware/driver/protocol/fake_protocol.hpp"
#include "ros_control_hardware/driver/protocol/yiyou_protocol.hpp"
#include "ros_control_hardware/driver/transport/loopback_transport.hpp"
#include "ros_control_hardware/driver/protocol_factory.hpp"

using namespace ros_control_hardware;

int main()
{
  // ------------------------------------------------------------
  // 测试 1：单位换算（位置 / 速度 / 力矩）
  // ------------------------------------------------------------
  JointModel model(1.0, 10.0, 0.1);

  const double device_position = model.toDevicePosition(0.5);
  const bool position_ok =
      std::fabs(model.toJointPosition(device_position) - 0.5) < 1e-9;

  const double device_velocity = model.toDeviceVelocity(2.0);
  const bool velocity_ok =
      std::fabs(model.toJointVelocity(device_velocity) - 2.0) < 1e-9;

  const double device_effort = model.toDeviceEffort(3.0);
  const bool effort_ok =
      std::fabs(model.toJointEffort(device_effort) - 3.0) < 1e-9;

  const bool model_ok = position_ok && velocity_ok && effort_ok;

  std::printf(
      "joint model: pos=%d vel=%d eff=%d (device_pos=%.4f joint_pos=%.4f) [%s]\n",
      position_ok, velocity_ok, effort_ok, device_position,
      model.toJointPosition(device_position), model_ok ? "ok" : "FAIL");

  // ------------------------------------------------------------
  // 测试 2：假电机收敛
  // ------------------------------------------------------------
  LoopbackTransport transport;
  FakeProtocol motor(transport, 0x01, 0.5);
  motor.setTimeStep(0.005);

  DeviceCommand command;
  command.position = 0.5;
  command.velocity = 0.0;
  command.effort = 0.0;
  command.kp = 40.0;
  command.kd = 2.0;

  double position = 0.0;
  int decoded_frames = 0;

  for (int step = 0; step < 1000; ++step) {
    motor.writeCommand(command);

    CanFrame frame;
    while (transport.receive(frame)) {
      DeviceState state;
      if (motor.decode(frame, state)) {
        position = state.position;
        ++decoded_frames;
      }
    }
  }

  std::printf("fake motor: target=0.5 final=%.4f frames=%d\n",
              position, decoded_frames);

  const bool motor_ok =
      std::fabs(position - 0.5) < 0.02 && decoded_frames == 1000;

  // ------------------------------------------------------------
  // 测试 3：ENCOS 编码 / 解码
  // ------------------------------------------------------------
  bool encos_ok = false;
  {
    LoopbackTransport encos_transport;
    EncosLimits limits;
    EncosProtocol encos(encos_transport, 0x03, limits);

    DeviceCommand encos_command;
    encos_command.position = 0.5;
    encos_command.kp = 40.0;
    encos_command.kd = 2.0;
    encos.writeCommand(encos_command);

    const bool sent_ok = encos_transport.sentFrames().size() == 1 &&
                         encos_transport.sentFrames()[0].id == 0x03 &&
                         encos_transport.sentFrames()[0].len == 8;

    CanFrame reply;
    reply.id = 0x03;
    reply.len = 8;
    const int pos_i =
        EncosProtocol::floatToUint(0.5, limits.pos_min, limits.pos_max, 16);
    reply.data[1] = static_cast<uint8_t>(pos_i >> 8);
    reply.data[2] = static_cast<uint8_t>(pos_i & 0xFF);
    reply.data[6] = 50;
    reply.data[7] = 50;

    DeviceState state;
    const bool decode_ok = encos.decode(reply, state);
    encos_ok = sent_ok && decode_ok && std::fabs(state.position - 0.5) < 0.002;

    std::printf("encos: sent=%d decode=%d pos=%.4f [%s]\n",
                sent_ok, decode_ok, state.position, encos_ok ? "ok" : "FAIL");
  }

  // ------------------------------------------------------------
  // 测试 4：意优编码 / 解码
  // ------------------------------------------------------------
  bool yiyou_ok = false;
  {
    LoopbackTransport yiyou_transport;
    YiyouLimits limits;
    YiyouProtocol yiyou(yiyou_transport, 0x01, limits);

    std::string error;
    yiyou.configure(error);
    const std::size_t after_config = yiyou_transport.sentFrames().size();
    const bool config_ok = after_config == 3;

    DeviceCommand yiyou_command;
    yiyou_command.position = 0.5;
    yiyou_command.kp = 40.0;
    yiyou_command.kd = 2.0;
    yiyou.writeCommand(yiyou_command);

    const CanFrame & sent = yiyou_transport.sentFrames().back();
    const bool frame_ok =
        yiyou_transport.sentFrames().size() == after_config + 1 &&
        sent.id == 0x01 && sent.len == 6 &&
        sent.data[0] == YiyouProtocol::kCmdFastWrite &&
        sent.data[1] == YiyouProtocol::kRegTargetTorque;

    // 手工造一帧"当前位置 = 0.5 rad"的 READ_REPLY
    CanFrame reply;
    reply.id = 0x01;
    reply.len = 6;
    reply.data[0] = YiyouProtocol::kCmdReadReply;
    reply.data[1] = YiyouProtocol::kRegCurrentPosition;

    constexpr double kTwoPi = 6.28318530717958647692;
    const auto pulses = static_cast<uint32_t>(0.5 / kTwoPi * 65536.0);
    reply.data[2] = static_cast<uint8_t>((pulses >> 24) & 0xFF);
    reply.data[3] = static_cast<uint8_t>((pulses >> 16) & 0xFF);
    reply.data[4] = static_cast<uint8_t>((pulses >> 8) & 0xFF);
    reply.data[5] = static_cast<uint8_t>(pulses & 0xFF);

    DeviceState state;
    const bool decode_ok = yiyou.decode(reply, state);
    yiyou_ok = config_ok && frame_ok && decode_ok &&
               std::fabs(state.position - 0.5) < 0.01;

    std::printf("yiyou: config=%zu frame=%d decode=%d pos=%.4f [%s]\n",
                after_config, frame_ok, decode_ok, state.position,
                yiyou_ok ? "ok" : "FAIL");
  }
    bool factory_ok = false;
  {
    registerBuiltinProtocols();

    LoopbackTransport factory_transport;
    MotorSpec spec;
    spec.type = "encos";
    spec.can_id = 0x02;

    auto encos = ProtocolFactory::create("encos", factory_transport, spec);
    auto yiyou = ProtocolFactory::create("yiyou", factory_transport, spec);
    auto unknown = ProtocolFactory::create("unknown", factory_transport, spec);

    factory_ok = (encos != nullptr) && (yiyou != nullptr) && (unknown == nullptr);

    std::printf("factory: encos=%d yiyou=%d unknown=%d [%s]\n",
                encos != nullptr ? 1 : 0,
                yiyou != nullptr ? 1 : 0,
                unknown != nullptr ? 1 : 0,
                factory_ok ? "ok" : "FAIL");
  }
  // ------------------------------------------------------------
  // 总结
  // ------------------------------------------------------------
  const bool all_ok = model_ok && motor_ok && encos_ok && yiyou_ok && factory_ok;
  std::printf("%s\n", all_ok ? "PASS" : "FAIL");
  return all_ok ? 0 : 1;
}
