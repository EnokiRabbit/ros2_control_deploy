# ros_control 部署框架

用 `ros2_control` 从 0 到 1 实现人形机器人部署栈。
本目录是正在实现中的工作区，进度见下方「实现进度」，契约见 `docs/`。

> 行为参考：`../develop_ws`（旧 joint_node / infer / imu / joy 实现）

## ros2_control 自带什么，我们要写什么

这是最容易搞混的地方：**ros2_control 只提供"框架 + 控制循环 + 接口抽象"，
不提供你的电机协议、运动学、策略和消息。**

### 已经自带（apt 装好，直接用，别重写）

| 能力 | 组件 | 说明 |
|---|---|---|
| 控制主循环 | `controller_manager` / `ros2_control_node` | 固定频率调用所有控制器的 `update()`，再调硬件的 `read()`/`write()` |
| 硬件抽象 | `hardware_interface::SystemInterface` | 规定 `on_init/on_configure/on_activate/read/write` 这套生命周期 |
| 接口句柄 | `hardware_interface::StateInterface` / `CommandInterface` | 状态/命令的 `double*` 绑定与导出 |
| 控制器基类 | `controller_interface::ControllerInterface` | 规定 `update()`、接口配置、生命周期 |
| 状态发布 | `joint_state_broadcaster` | 订阅硬件 state 接口，发 `/joint_states` |
| 控制器加载/切换 | `spawner` / `unspawner`、`ros2 control` CLI | 运行时加载、激活、切换控制器 |
| 标准控制器 | `joint_trajectory_controller`、`forward_command_controller`、`effort_controllers`、`velocity_controllers`、`position_controllers`、`pid_controller` | 常规关节控制够用 |
| 无硬件仿真 | `mock_components/GenericSystem` | 位置/速度/力矩三种接口可直接模拟 |
| 实时工具 | `realtime_tools` | `RealtimeBuffer`、`RealtimePublisher`，避免锁 |
| 模型发布 | `robot_state_publisher` | 读 URDF 发 TF |
| 启动编排 | `launch` / `launch_ros` | 组合节点与参数 |

### 必须自己写（ros2_control 没有）

| 要写的东西 | 落在哪个包 | 为什么不能直接用现成的 |
|---|---|---|
| MIT 关节命令消息（pos/vel/effort/kp/kd 一起下发） | `ros_control_msgs` | 标准只有 `JointState`/`JointTrajectory`，没有"变刚度 + 前馈力矩"的消息 |
| 电机遥测、推理状态、模式切换/置零服务 | `ros_control_msgs` | 全是本项目语义 |
| CAN 收发 + 22 个电机协议 + 14 关节映射 + 下肢 3×3 运动学 + 安全回退 | `ros_control_hardware` | `SystemInterface` 只是空壳，`read/write` 里做什么完全由你决定 |
| 置零服务、电机遥测发布 | `ros_control_hardware`（admin 部分） | 依赖驱动实例，必须和硬件同进程 |
| 无 CAN 仿真（可选） | `ros_control_hardware`（MockSystem） | `GenericSystem` 不导出自定义接口 `kp/kd`，我们的控制器会加载失败 |
| **MIT 控制器**（五接口 + 超时阻尼） | `ros_control_controllers` | 见下方"为什么必须自己写控制器" |
| 机器人模型 | `ros_control_description` | URDF 是你的机器人，`robot_state_publisher` 只负责发布 |
| ONNX 推理 + 255 维观测 + 模式状态机 + 看门狗 | `ros_control_inference` | 策略相关，无任何现成组件 |
| IMU 串口协议解析 | `ros_control_imu` | `imu_sensor_broadcaster` 只负责发，不含 Xsens/Yesense 协议 |
| 手柄按键映射（LB/A/B/X） | `ros_control_joy` | `joy` 只把 `/dev/input` 转成 `sensor_msgs/Joy` |
| 启动编排 + 参数 | `ros_control_bringup` | launch 组合方式是本项目约定 |
| 对外 API | `ros_control_sdk` / `ros_control_sdk_py` | 无标准 SDK |

### 为什么必须自己写 MIT 控制器

现成控制器都不满足 RL 部署：

- `forward_command_controller`：只转发**一个**接口（position 或 effort），没有 kp/kd。
- `effort_controllers/*`：只有 effort，没有刚度/阻尼。
- `joint_trajectory_controller`：轨迹插值 + PID，面向慢速轨迹，不适合 200Hz 变刚度流式指令。

RL 策略每 5ms 下发一次 `(q_target, kp, kd, τ_ff)`，所以必须写一个把五个命令接口一起写、
并在超时后自动降为阻尼的控制器。

### 一句话分工

```
ros2_control 给你：循环、生命周期、接口、状态发布、控制器加载
你给 ros2_control：硬件 read/write 的实现、控制器逻辑、机器人模型、消息、策略、SDK
```

## 包结构

```
src/
├── ros_control_msgs         消息/服务
├── ros_control_hardware     SystemInterface（CAN + 电机 + 运动学）+ Mock
├── ros_control_controllers  mit_joint_controller
├── ros_control_description  URDF/xacro + <ros2_control> 标签
├── ros_control_inference    ONNX 推理节点
├── ros_control_imu          IMU 节点
├── ros_control_joy          手柄节点
├── ros_control_bringup      launch + yaml
├── ros_control_sdk          C++ SDK + CLI
└── ros_control_sdk_py       Python SDK
```

## 实现顺序

```
① msgs → ② hardware 只读 → ③ hardware 写 → ④ controller
       → ⑤ URDF/bringup+mock → ⑥ inference → ⑦ imu/joy → ⑧ SDK
```

每步先 `colcon build` 通过，再进下一步。细节见 `docs/IMPLEMENTATION.md`。

## 实现进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| 0 | 环境、工作区、colcon/VS Code | ✅ |
| 1 | `ros_control_msgs`：4 msg + 2 srv | ✅ |
| 2.1 | `ros_control_hardware`：插件骨架、5 command / 3 state 接口 | ✅ |
| 2.2 | 驱动层：接口 + 回环传输 + 单位换算 | ✅ |
| 2.2b | 假电机 + `driver_test` | ✅ |
| 2.3 | ENCOS / 意优真实协议 | ✅ |
| 2.3b | 意优力矩协议 | ✅ |
| 2.4 | 协议工厂 `ProtocolFactory` | ✅ |
| 2.5 | `SystemHardware` 接入驱动层 | ✅ 已用最小 URDF 验证 |
| 2.6 | SocketCAN `CanTransport` | ⬜ |
| 3 | 控制器 `mit_joint_controller` | ⬜ |
| 4 | URDF + bringup + mock | 🔶 最小 URDF/launch 已跑通，完整模型待做 |
| 5 | 推理节点 | ⬜ |
| 6 | IMU / 手柄 | ⬜ |
| 7 | SDK | ⬜ |

## 目录结构

`✅` 已实现，`⬜` 待实现。

```
src/ros_control_hardware/
├── package.xml
├── CMakeLists.txt
├── ros_control_hardware_plugins.xml
├── include/ros_control_hardware/
│   ├── system_hardware.hpp                     ✅ 插件入口
│   └── driver/
│       ├── transport/
│       │   ├── transport.hpp                   ✅ 传输接口（搬字节）
│       │   ├── loopback_transport.hpp           ✅ 回环（测试替身）
│       │   └── can_transport.hpp                ⬜ SocketCAN
│       ├── protocol/
│       │   ├── motor_protocol.hpp               ✅ 协议接口（编解码）
│       │   ├── fake_protocol.hpp                ✅ 假电机（测试替身）
│       │   ├── encos_protocol.hpp               ⬜ ENCOS
│       │   └── yiyou_protocol.hpp               ⬜ 意优
│       ├── joint/
│       │   └── joint_model.hpp                  ✅ 单位换算
│       └── protocol_factory.hpp                 ✅ 按 type 创建协议
├── src/
│   ├── system_hardware.cpp                      ✅ 生命周期 + 接口导出
│   └── driver/
│       ├── transport/loopback_transport.cpp     ✅
│       ├── transport/can_transport.cpp          ⬜
│       ├── protocol/fake_protocol.cpp           ✅
│       ├── protocol/encos_protocol.cpp          ⬜
│       ├── protocol/yiyou_protocol.cpp          ⬜
│       └── protocol_factory.cpp                 ✅
└── test/
    └── driver_test.cpp                          ✅
```

### 驱动层分层规则

| 目录 | 职责 | 变化频率 |
|---|---|---|
| `transport/` | 字节怎么进出总线 | 换总线（CAN→EtherCAT）时变 |
| `protocol/` | 字节是什么意思 | 换电机品牌时变 |
| `joint/` | 设备值 ↔ 关节值 | 只改参数（方向/减速比/零偏） |
| `protocol_factory` | 按 `type` 字符串创建 | 加品牌时注册一行 |

**生产代码**（进 `libros_control_hardware.so`）：`system_hardware.cpp`、`can_transport.cpp`、真实协议。
**测试替身**（只进 `driver_test`）：`loopback_transport.cpp`、`fake_protocol.cpp`。

## 待回头学习（SDK 完成后）

电机协议的细节先跳过，等 SDK 写完、整条链路跑通后再回来补：

- [ ] ENCOS type-1 回包每个字节的含义（错误码、位置、速度、电流、两个温度）
- [ ] 意优寄存器表：`WORK_MODE` / `ENABLE_STATE` / `TARGET_TORQUE` / `CURRENT_*`
- [ ] 量化位宽与量程：为什么 kp 12 位、kd 9 位、位置 16 位
- [ ] 方向 `direction`、减速比 `gear_ratio`、零偏 `offset` 的标定方法
- [ ] 速度/力矩换算为什么和位置不同（力矩是乘减速比，速度是除）
- [ ] `requestState` 与自动回包的时序差异（ENCOS vs 意优）
- [ ] 用 `candump` / `cansend` 抓包验证字节
- [ ] 各型号的真实量程（kp/kd/pos/spd/current/kt）和错误码含义
- [ ] 过温、过流、堵转时电机回什么、代码该怎么处理

## 下一步

- [x] 最小 URDF（`test/minimal.urdf`）验证插件加载
- [ ] `controllers.yaml` + `joint_state_broadcaster` 看到 `/joint_states`
- [ ] `mit_joint_controller`（第 3 步）
- [ ] 完整 URDF + bringup（第 4 步）
- [ ] 推理节点（第 5 步）
- [ ] IMU / 手柄（第 6 步）
- [ ] SDK（第 7 步）


## 约定（先定死，少返工）

### 受控关节（14，策略顺序）

```
left_shoulder_pitch_joint   right_shoulder_pitch_joint
right_hip_yaw_joint         left_hip_yaw_joint
left_hip_roll_joint         left_hip_pitch_joint
left_knee_joint             left_ankle_pitch_joint
left_ankle_roll_joint       right_hip_roll_joint
right_hip_pitch_joint       right_knee_joint
right_ankle_pitch_joint     right_ankle_roll_joint
```

### 控制模式

| 值 | 模式 | 行为 |
|---|---|---|
| 0 | IDLE | 不追目标，控制器超时回阻尼 |
| 1 | CROUCH | 发蹲姿，高刚度 |
| 2 | INFERENCE | 策略闭环 |
| 3 | DAMPING | kp=0，只留 kd |

### 话题

- `/joint_states`（`sensor_msgs/JointState`，broadcaster 发）
- `/mit_joint_controller/commands`（`ros_control_msgs/MitJointCommand`）
- `/cmd_vel`（`geometry_msgs/Twist`）、`/control_mode`（`std_msgs/UInt8`）
- `/imu`（`sensor_msgs/Imu`）

## 环境

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <包名>
source install/setup.bash
```

`scripts/build.sh` 是封装，`scripts/env.sh` 给非交互 shell 用。
