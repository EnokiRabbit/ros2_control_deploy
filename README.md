# ros2_control 机器人部署框架

基于 **ROS 2 Jazzy + ros2_control** 的人形机器人部署栈：从 CAN 电机驱动、关节控制，到策略推理与对外 SDK，全部按 ros2_control 的插件机制组织。

目标是让同一套控制器、推理节点和 SDK 可以运行在：

- **真机**：真实 CAN 总线 + 真实电机
- **vcan**：Linux 虚拟 CAN + 假电机进程
- **mock**：ROS 自带 `mock_components/GenericSystem`

切换方式只是改 URDF 里的 `<plugin>` 和 `<param>`，上层代码不变。

> 当前仓库是通用框架，机器人模型使用占位的 `robot.urdf`。真实机器人的 URDF / mesh 不放在仓库里。

---

## 目录

- [特性](#特性)
- [系统架构](#系统架构)
- [仓库结构](#仓库结构)
- [快速开始](#快速开始)
- [配置说明](#配置说明)
- [测试](#测试)
- [设计原则](#设计原则)
- [当前进度](#当前进度)
- [路线图](#路线图)
- [常见问题](#常见问题)
- [License](#license)

---

## 特性

| 能力 | 说明 |
|---|---|
| 硬件插件 | `SystemInterface` 实现，支持多关节、多总线 |
| 真实 CAN | `CanTransport`，基于 Linux SocketCAN，非阻塞读写 |
| 虚拟 CAN | `vcan0` 上完整跑通，无需真实电机 |
| 电机协议 | ENCOS（MIT 8 字节）、意优（寄存器 / 力矩千分比） |
| 协议扩展 | `ProtocolFactory` 按字符串创建，新增品牌只加一个类 |
| 单位换算 | `JointModel` 统一处理方向、减速比、零偏、力矩换算 |
| MIT 控制器 | 一次写 position / velocity / effort / kp / kd，超时降阻尼 |
| 状态发布 | `joint_state_broadcaster` 发布 `/joint_states` |
| 仿真替身 | `fake_encos_motor` 在 vcan 上模拟电机回包 |
| 配置驱动 | 关节、总线、电机类型、ID、减速比全部来自 URDF |

---

## 系统架构

### 运行时进程

```
┌──────────────────────────────────────────────────────────────┐
│ ros2_control_node（controller_manager）                      │
│                                                              │
│  controller_manager                                          │
│   ├── joint_state_broadcaster  ──► /joint_states             │
│   └── mit_joint_controller     ◄── /mit_joint_controller/commands
│                                                              │
│  ResourceManager                                             │
│   └── SystemHardware                                         │
│        ├── CanTransport("can0" / "vcan0")                    │
│        ├── EncosProtocol / YiyouProtocol                     │
│        └── JointModel                                        │
└──────────────────────────────────────────────────────────────┘
             │ CAN 帧
             ▼
     真实电机 / vcan0 上的 fake_encos_motor
```

### 一个控制周期

```
controller_manager
  ├─ read()     硬件读 CAN → 解码 → 换算 → state 接口
  ├─ update()   控制器读 state、写 command 接口
  └─ write()    command 接口 → 换算 → 编码 → 发 CAN
```

### 数据流

```
/joint_states ──► inference（待实现）──► /mit_joint_controller/commands
                                              │
                                              ▼
                                   mit_joint_controller
                                              │
                                              ▼
                                   SystemHardware::write()
                                              │
                                              ▼
                                   CanTransport → CAN / vcan
                                              │
                                              ▼
                                         电机 / 假电机
                                              │
                                              ▼
                                   SystemHardware::read()
                                              │
                                              ▼
                                   joint_state_broadcaster
                                              │
                                              ▼
                                        /joint_states
```

---

## 仓库结构

```
src/
├── ros_control_msgs/            消息与服务定义
│   ├── msg/MitJointCommand.msg
│   ├── msg/MotorState.msg
│   ├── msg/MotorStateArray.msg
│   ├── msg/InferenceStatus.msg
│   ├── srv/SwitchMode.srv
│   └── srv/SetZeroPosition.srv
│
├── ros_control_hardware/        硬件插件 + 驱动层
│   ├── include/ros_control_hardware/
│   │   ├── system_hardware.hpp
│   │   └── driver/
│   │       ├── transport/       Transport 接口、Loopback、CanTransport
│   │       ├── protocol/        MotorProtocol、ENCOS、意优、Fake
│   │       ├── joint/           JointModel
│   │       └── protocol_factory.hpp
│   ├── src/
│   ├── test/
│   │   ├── driver_test.cpp          协议 / 换算单元测试
│   │   ├── can_transport_test.cpp   SocketCAN 收发测试
│   │   └── fake_encos_motor.cpp     vcan 假电机
│   └── ros_control_hardware_plugins.xml
│
├── ros_control_controllers/     MIT 控制器插件
│   └── src/mit_joint_controller.cpp
│
├── ros_control_description/     URDF
│   └── urdf/
│       ├── minimal.urdf          2 关节回环调试
│       ├── minimal_vcan.urdf     2 关节 vcan
│       └── robot.urdf            14 关节通用模型（占位参数）
│
├── ros_control_bringup/         launch + 控制器配置
│   ├── config/controllers.yaml
│   └── launch/
│       ├── minimal.launch.py
│       ├── minimal.vcan.launch.py
│       └── robot.launch.py
│
└── (计划中)
    ├── ros_control_inference/
    ├── ros_control_imu/
    ├── ros_control_joy/
    └── ros_control_sdk/
```

---

## 快速开始

### 1. 环境

- Ubuntu 24.04
- ROS 2 Jazzy
- `can-utils`（vcan 调试）
- 可选：`python3-can`

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. 编译

```bash
cd ~/ros_control_deploy
colcon build --symlink-install
source install/setup.bash
```

### 3. 创建 vcan（需要 sudo）

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0
```

### 4. 启动假电机

```bash
./build/ros_control_hardware/fake_encos_motor vcan0
```

输出：

```
fake ENCOS motor on vcan0, ids 1..14
```

### 5. 启动 ros2_control

另一个终端：

```bash
cd ~/ros_control_deploy
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch ros_control_bringup robot.launch.py
```

预期日志：

```
joint left_knee_joint: bus=vcan0 type=encos id=1
...
SystemHardware: 14 joints, 1 buses
Loaded hardware 'RobotSystem'
mit_joint_controller    active
joint_state_broadcaster active
```

### 6. 发命令

第三个终端：

```bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic pub -r 20 /mit_joint_controller/commands \
  ros_control_msgs/msg/MitJointCommand \
  '{name: [left_knee_joint], position: [0.2], velocity: [0.0], effort: [0.0], kp: [40.0], kd: [2.0]}'
```

### 7. 看状态

```bash
ros2 topic echo /joint_states --once
```

预期 `left_knee_joint` 的 `position` 接近 `0.2`，其他关节接近 `0`。

---

## 配置说明

### 硬件参数（URDF）

每个受控关节在 `<ros2_control>` 里声明：

```xml
<joint name="left_knee_joint">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>
  <command_interface name="kp"/>
  <command_interface name="kd"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>

  <param name="bus">vcan0</param>
  <param name="motor_type">encos</param>
  <param name="can_id">1</param>
  <param name="direction">1.0</param>
  <param name="gear_ratio">1.0</param>
  <param name="offset">0.0</param>
</joint>
```

| 参数 | 含义 | 真机取值 |
|---|---|---|
| `bus` | CAN 接口名 | `can0` / `can1` / `can2` |
| `motor_type` | 协议类型 | `encos` / `yiyou` |
| `can_id` | 电机节点号 | 按电机拨码/配置 |
| `direction` | 安装方向 | `1.0` 或 `-1.0` |
| `gear_ratio` | 减速比 | 按电机型号 |
| `offset` | 零位偏置 | 标定值 |

> `robot.urdf` 里的值都是占位值，上真机前必须替换。

### 控制器（controllers.yaml）

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100          # VM 用 100；真机建议 200

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    mit_joint_controller:
      type: ros_control_controllers/MitJointController

mit_joint_controller:
  ros__parameters:
    joints:
      - left_shoulder_pitch_joint
      - ...
    command_timeout: 0.5
    damping_kd: 0.0
```

- `joints` 必须和 URDF 里的受控关节一致
- `command_timeout`：超过这个时间没收到新命令，控制器切阻尼
- `damping_kd`：超时后的阻尼系数，`0` 表示完全自由（对齐旧系统）

### 启动文件

| launch | 用途 |
|---|---|
| `minimal.launch.py` | 2 关节，回环 |
| `minimal.vcan.launch.py` | 2 关节，vcan |
| `robot.launch.py` | 14 关节，vcan / 真机（取决于 URDF 的 bus） |

---

## 测试

### 1. 协议 / 换算单元测试

```bash
./build/ros_control_hardware/driver_test
```

覆盖：

```
JointModel 往返换算
FakeProtocol 收敛
ENCOS 编码 / 解码
意优编码 / 解码
协议工厂
```

### 2. SocketCAN 收发

```bash
./build/ros_control_hardware/can_transport_test
```

另一个终端：

```bash
candump vcan0
cansend vcan0 456#AABBCCDDEEFF1122
```

### 3. vcan 闭环

见[快速开始](#快速开始)。

### 4. 观察 CAN 帧

```bash
candump vcan0
```

能看到 14 个 ID 的命令帧；假电机开启后，还能看到状态帧。

---

## 设计原则

### 1. 按职责分层

```
SystemHardware  调度：生命周期、read/write
CanTransport    搬字节：socketcan / 回环
Protocol        编解码：ENCOS / 意优
JointModel      换算：方向 / 减速比 / 零偏
Controller      决策：目标 / 增益 / 超时
```

每层只懂自己那一小块，换电机不影响控制器，换控制器不影响驱动。

### 2. 配置驱动

```
换电机品牌  → 新增 Protocol + 注册一行 + 改 yaml
换减速比    → 只改 URDF 参数
换总线      → 只改 bus 参数
换控制器    → 只改 controllers.yaml
```

### 3. 仿真也是插件

- `vcan + fake_encos_motor`：验证真实 socketcan 路径
- `mock_components/GenericSystem`：验证控制器逻辑
- 以后接 MuJoCo / Gazebo 时，也做成硬件插件

生产代码里不写 `if (simulation)` 分支。

### 4. 测试替身不进生产库

`LoopbackTransport`、`FakeProtocol`、`fake_encos_motor` 只用于测试，不安装到机器人。

---

## 当前进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| 0 | 环境、工作区 | ✅ |
| 1 | `ros_control_msgs` | ✅ |
| 2.1 | 硬件插件骨架 | ✅ |
| 2.2 | 驱动层接口、回环、单位换算 | ✅ |
| 2.2b | 假电机 + `driver_test` | ✅ |
| 2.3 | ENCOS / 意优协议 | ✅ |
| 2.4 | 协议工厂 | ✅ |
| 2.5 | `SystemHardware` 接入 | ✅ |
| 2.6 | `CanTransport`（SocketCAN） | ✅ |
| 2.7 | vcan 假电机闭环 | ✅ |
| 3 | `mit_joint_controller` | ✅ |
| 4 | 14 关节通用 URDF + bringup | ✅ |
| 5 | 推理节点 | ⬜ |
| 6 | IMU / 手柄 | ⬜ |
| 7 | SDK | ⬜ |
| 8 | 真机联调 | ⬜ |

已验证：

```
14 关节 vcan 闭环
发 left_knee = 0.2 → /joint_states 的 left_knee ≈ 0.198
其余关节保持 0
```

---

## 路线图

### 近期

1. 填真实电机参数（bus / can_id / gear_ratio / direction / offset）
2. 推理节点
   - 先做假策略（固定站姿 / 正弦），跑通话题闭环
   - 再接 ONNX
3. IMU 节点
4. 手柄节点
5. SDK（C++ / Python）
6. 真机 CAN 联调

### 推理节点接口（计划）

```
订阅：
  /joint_states
  /imu
  /cmd_vel
  /control_mode

发布：
  /mit_joint_controller/commands
  /inference_status

频率：
  50 Hz 推理 / 200 Hz 下发
```

---

## 常见问题

### 1. VM 上 overrun 警告

```
Overrun detected! ... loop took 5.2 ms
```

原因：VM 没有实时调度权限，200Hz 周期 5ms 容易被抢占。

处理：

- VM 上用 `update_rate: 100`
- 真机用 PREEMPT_RT 内核 + RT 权限 + 200Hz
- 参考 `controllers.yaml` 里的注释

### 2. vcan0 不存在

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 3. `/joint_states` 是 0

检查：

```bash
candump vcan0            # 有没有回包
ps aux | grep fake_encos # 假电机在不在
ros2 control list_controllers
```

假电机没启动时，`read()` 收不到状态，`/joint_states` 会一直是 0。

### 4. `ros2 topic pub` 参数被 shell 拆开

YAML 部分用单引号包起来：

```bash
ros2 topic pub ... '{name: [left_knee_joint], position: [0.2], ...}'
```

### 5. 控制器激活失败

```
Could not find command interface '...'
```

检查：

- URDF 的关节名和 `controllers.yaml` 的 `joints` 是否一致
- URDF 是否声明了 `position/velocity/effort/kp/kd`
- 硬件插件是否导出了这些接口

---

## License

尚未选择。使用前请先补充 `LICENSE` 文件，并确认机器人模型、mesh 和电机协议的再分发许可。
