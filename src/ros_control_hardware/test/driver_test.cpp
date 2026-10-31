// 驱动层验收：不需要 CAN，也不需要真电机。
//
// 覆盖：
//   1. JointModel 的往返换算
//   2. FakeProtocol 在 PD 控制下能否收敛到目标

#include <cmath>
#include <cstdio>

#include "ros_control_hardware/driver/joint/joint_model.hpp"
#include "ros_control_hardware/driver/protocol/fake_protocol.hpp"
#include "ros_control_hardware/driver/transport/loopback_transport.hpp"

using namespace ros_control_hardware;

int main()
{
  // ------------------------------------------------------------
  // 测试 1：单位换算
  //   joint=0.5, gear=10, offset=0.1
  //   device = (0.5 - 0.1) * 10 = 4.0
  //   joint  = 4.0 / 10 + 0.1  = 0.5
  // ------------------------------------------------------------
  JointModel model(1.0, 10.0, 0.1);
  const double device = model.toDevice(0.5);
  const bool model_ok = std::fabs(model.toJoint(device) - 0.5) < 1e-9;

  std::printf("joint model: joint=0.5 -> device=%.4f -> joint=%.4f  [%s]\n",
              device, model.toJoint(device), model_ok ? "ok" : "FAIL");

  // ------------------------------------------------------------
  // 测试 2：假电机收敛
  //   目标 0.5 rad，kp=40，kd=2，惯量 0.5，步长 5ms
  //   跑 1000 步 = 5 秒
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
    // 下发一步命令：假电机内部积分 5ms，并 push 一帧回包
    motor.writeCommand(command);

    // 取空队列里的所有回包
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

  std::printf("%s\n", (model_ok && motor_ok) ? "PASS" : "FAIL");
  return (model_ok && motor_ok) ? 0 : 1;
}
