# ros_control 部署框架

用 `ros2_control` 从 0 到 1 实现人形机器人部署栈。
本目录是**空的骨架 + 契约**，代码由你按 `docs/IMPLEMENTATION.md` 的检查点一步步写，我只做讲解和 review。

> 参考项目（行为基准，可读不可抄）：`../develop_ws`
> 其中 `src/driver` 是旧的 joint_node/imu/joy 实现，`src/infer` 是旧推理节点。

## 目标栈

```
                    ┌──────────── controller_manager @200Hz ────────────┐
 /joint_states ◄────┤ joint_state_broadcaster                           │
                    │ mit_joint_controller ◄── ~/commands ◄──────────────┼── infer_node (ONNX)
                    │        │ position/velocity/effort/kp/kd           │
                    │        ▼                                          │
                    │ <robot_hardware> (SystemInterface) ──► CAN0/1/2   │
                    └───────────────────────────────────────────────────┘
 imu_node ──► /imu ──► infer_node        joy_node ──► /cmd_vel, /control_mode
```

## 包结构

```
src/
├── ros_control_msgs         消息/服务
├── ros_control_hardware     ros2_control SystemInterface（CAN + 电机 + 运动学）
├── ros_control_controllers  mit_joint_controller
├── ros_control_description  URDF/xacro + <ros2_control> 标签
├── ros_control_inference    ONNX 推理节点
├── ros_control_imu          IMU 节点
├── ros_control_joy          手柄节点
├── ros_control_bringup      launch + yaml
├── ros_control_sdk          C++ SDK + CLI
└── ros_control_sdk_py       Python SDK
```

## 从哪开始

1. 读 `docs/ARCHITECTURE.md`：数据流、接口契约、安全分层。
2. 读 `docs/PACKAGE_CONTRACTS.md`：每个包要建哪些文件、参数、验收命令。
3. 按 `docs/IMPLEMENTATION.md` 的 9 个检查点写，**每个检查点先编译通过再往下**。

## 约定（先定死，少返工）

### 受控关节（14，策略顺序）

```
left_shoulder_pitch_joint
right_shoulder_pitch_joint
right_hip_yaw_joint
left_hip_yaw_joint
left_hip_roll_joint
left_hip_pitch_joint
left_knee_joint
left_ankle_pitch_joint
left_ankle_roll_joint
right_hip_roll_joint
right_hip_pitch_joint
right_knee_joint
right_ankle_pitch_joint
right_ankle_roll_joint
```

### 控制模式

| 值 | 模式 | 行为 |
|---|---|---|
| 0 | IDLE | 不追目标，控制器超时回阻尼 |
| 1 | CROUCH | 发蹲姿，高刚度 |
| 2 | INFERENCE | 策略闭环 |
| 3 | DAMPING | kp=0，只留 kd |

### 命名

- 硬件组件：`LocamotionSystem`（URDF `<ros2_control name=...>`）
- 命令话题：`/mit_joint_controller/commands`（`ros_control_msgs/MitJointCommand`）
- 状态话题：`/joint_states`（`sensor_msgs/JointState`，broadcaster 发）
- 速度命令：`/cmd_vel`（`geometry_msgs/Twist`）
- 模式话题：`/control_mode`（`std_msgs/UInt8`）

## 环境

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <包名>
source install/setup.bash
```

`scripts/build.sh` 是封装；`scripts/env.sh` 给非交互 shell 用。
