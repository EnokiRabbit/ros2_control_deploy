# 架构与接口契约

## 1. 数据流

```
CAN0/1/2 ──► hardware.read() ──► state interface (position/velocity/effort)
                                      │
                                      ▼
                            joint_state_broadcaster ──► /joint_states
                                      │
infer_node ◄── /joint_states, /imu, /cmd_vel, /control_mode
     │
     └── ros_control_msgs/MitJointCommand ──► /mit_joint_controller/commands
                                                  │
                                                  ▼
                                    mit_joint_controller.update()
                                                  │ 写 command interface
                                                  ▼
                                    hardware.write() ──► MIT 帧 ──► CAN
```

关键点：**CAN 收发只在 hardware 的 `read()`/`write()` 里**。
"关节节点"这个角色由 `hardware + mit_joint_controller` 取代，不再有单独的 joint_node。

## 2. 硬件组件契约

### 2.1 生命周期

| 阶段 | 做什么 |
|---|---|
| `on_init(info)` | 解析配置、校验 URDF 关节、分配缓冲；**不碰 CAN** |
| `on_configure` | 打开 CAN、建驱动实例、绑定、配置探测 |
| `on_activate` | 清队列、命令缓冲置 NaN、允许写 |
| `on_deactivate` | 发一次阻尼 |
| `on_cleanup` | 关 CAN、释放驱动 |
| `read(t, dt)` | 驱动解码 → 电机状态 → 运动学正解 → state interface |
| `write(t, dt)` | command interface → 运动学逆解 → MIT 帧 |

### 2.2 接口

每个受控关节导出：

```
command: position, velocity, effort, kp, kd     # kp/kd 是自定义接口名
state:   position, velocity, effort
```

`kp`/`kd` 是 MIT 控制和普通位置控制的核心区别，必须和指令一起下发。

### 2.3 读取语义

- 电机离线：**保持上一次数值**，不要写 0（会造成位置跳变）。
- 速度做死区处理（如 |v| < 0.006 → 0）。

### 2.4 写入语义

- 任一命令为 NaN/Inf → 走阻尼（说明控制器没激活或还没发指令）。
- kp/kd 负值钳到 0。
- 未受控的被动电机（手臂 8 轴）不参与 MIT，上电时按位置模式回零。
- 每轴再按电机型号的 `MitParamRange` 钳位一次（位置/速度/力矩/kp/kd）。

### 2.5 运动学

下肢 pitch 链 `hip_pitch + knee + ankle_pitch` 用 3×3 矩阵耦合：

- 关节→电机位置：`q_motor = M⁻¹ · q_joint`
- 电机→关节位置：`q_joint = M · q_motor`
- 力矩：`τ_motor = Mᵀ · τ_joint`，`τ_joint = (M⁻¹)ᵀ · τ_motor`

其余关节（hip_yaw / hip_roll / ankle_roll / shoulder_pitch）直驱透传。

## 3. 控制器契约（mit_joint_controller）

- 订阅 `~/commands`（`MitJointCommand`），`RealtimeBuffer` 传递到 `update()`。
- 消息按名字匹配，`name` 为空时按 `joints` 参数顺序。
- `update()`：读状态位置 → 判断新鲜度 → 写命令 或 写阻尼。
- 超时（默认 0.3s）→ `kp=0, kd=damping_kd, effort=0, position=当前测量`。
- `on_deactivate` 也必须写阻尼（控制器卸载时接口不再被写，保留高刚度很危险）。
- 要求消息覆盖全部关节（名字或位置式），否则拒绝。

## 4. 安全分层

| 层 | 机制 |
|---|---|
| 控制器 | 命令超时 → 阻尼 |
| 控制器 | 消息校验（长度/名字/有限值） |
| 硬件 | 命令 NaN → 阻尼 |
| 硬件 | 生命周期卸载 → 阻尼 |
| 驱动 | 离线跳过、按型号钳位 |
| 运动学 | 耦合关节力矩限幅 |
| 标定 | 置零期间暂停 read/write |

**不兜底**：`ros2_control_node` 整体卡死时不会自动失能，必须靠急停/掉电。

## 5. 话题与服务

| 名称 | 类型 | 方向 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 出 |
| `/mit_joint_controller/commands` | `ros_control_msgs/MitJointCommand` | 入 |
| `/cmd_vel` | `geometry_msgs/Twist` | 入 |
| `/control_mode` | `std_msgs/UInt8` | 入 |
| `/imu` | `sensor_msgs/Imu` | 入 |
| `/motor_states` | `ros_control_msgs/MotorStateArray` | 出（诊断） |
| `~/switch_mode` | `ros_control_msgs/SwitchMode` | 服务 |
| `~/set_zero_position` | `ros_control_msgs/SetZeroPosition` | 服务 |

`~` 表示所属节点的私有命名空间。
