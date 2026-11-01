// 驱动层验收：不需要 CAN，也不需要真电机。
//
// 覆盖：
//   1. JointModel 的往返换算
//   2. FakeProtocol 在 PD 控制下能否收敛到目标
//   3. EncosProtocol 的 8 字节编码 / type-1 解码

#include <cmath>
#include <cstdio>

#include "ros_control_hardware/driver/joint/joint_model.hpp"
#include "ros_control_hardware/driver/protocol/encos_protocol.hpp"
#include "ros_control_hardware/driver/protocol/fake_protocol.hpp"
#include "ros_control_hardware/driver/transport/loopback_transport.hpp"

using namespace ros_control_hardware;

int main()
{
  // ------------------------------------------------------------
  // 测试 1：单位换算
  // ------------------------------------------------------------
  JointModel model(1.0, 10.0, 0.1);
  const double device = model.toDevice(0.5);
  const bool model_ok = std::fabs(model.toJoint(device) - 0.5) < 1e-9;

  std::printf("joint model: joint=0.5 -> device=%.4f -> joint=%.4f  [%s]\n",
              device, model.toJoint(device), model_ok ? "ok" : "FAIL");

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
  // 总结
  // ------------------------------------------------------------
  const bool all_ok = model_ok && motor_ok && encos_ok;
  std::printf("%s\n", all_ok ? "PASS" : "FAIL");
  return all_ok ? 0 : 1;
}
