# 实现检查点（每步先编译通过再往下）

## 0. 环境

```bash
sudo apt install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-realtime-tools \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  libyaml-cpp-dev libeigen3-dev python3-yaml can-utils
```

> 版本要一致：如果 `ros2_control_node` 报 `undefined symbol ... diagnostic_updater`，
> 说明 ROS 包半升级了，先 `sudo apt upgrade -y` 统一版本。

## 1. 消息包（半天）

做：`ros_control_msgs` 的 4 个 msg + 2 个 srv。

过：`colcon build --packages-select ros_control_msgs`，`ros2 interface show` 能打印字段和模式常量。

## 2. 硬件只读通道（1-2 天）

做：配置解析 + CAN 打开 + 驱动实例 + `read()` + `export_state_interfaces()`。

要点：

- 22 电机分 3 路 CAN，按 `(bus, type)` 分组建驱动。
- 关节映射按名字，配置里 `motors[].joint`。
- 离线关节保持上次值。

过：`ros2 control list_hardware_interfaces` 看到 14×3 state；真机 `ros2 topic echo /motor_states --once` 数值合理（手掰关节方向正确）。

## 3. 硬件写通道（1 天，台架）

做：`write()` + NaN 判定 + 阻尼 + `enable_write` 开关 + 被动电机回零。

安全：**机器人吊起/腿部卸载**，先单关节 `kp=20, kd=1`，确认方向与限幅。

过：给一个关节发小目标，实际位置跟随且不过冲。

## 4. 控制器（半天）

做：`mit_joint_controller`，`RealtimeBuffer` + 超时阻尼 + 消息校验。

过：mock 下 `ros2 control list_controllers` 为 active；停发命令 0.3s 后仍稳（阻尼）。

## 5. URDF + bringup + mock（半天）

做：xacro（真实限位）、`controllers.yaml`、`mock.launch.py`。

过：

```bash
ros2 launch ros_control_bringup mock.launch.py
ros2 topic hz /joint_states          # ≈200
```

## 6. 推理节点（2 天）

做：ONNX Runtime + 观测构造 + 模式状态机 + 看门狗。

坑：

- 观测项顺序/帧顺序必须和训练一致，差一项整个策略就废。
- 进入 INFERENCE 时从当前姿态爬升（ramp），不要一步跳目标。
- 关节数据超过 0.3s 没更新 → 自动回 DAMPING。

过：mock 下策略输出稳定，`/joint_states` 不抖；真机吊起时关节跟随柔和。

## 7. IMU / 手柄（半天）

做：`imu_node`（`sensor_msgs/Imu`）、`joy_node`（`cmd_vel` + `control_mode`）。

过：`ros2 topic hz /imu` ≈100；按 X 后 `/control_mode=2`。

## 8. SDK（1 天）

做：C++ `Robot` 类 + CLI；Python 同名 API。

过：`ros2 run ros_control_sdk ros_control_cli status` 打印 14 关节；`walk --vx 0.2 --duration 2` 期间 `/cmd_vel` 有输出。

## 9. 收尾（半天）

- 置零服务（按 CAN 总线，阻塞式，执行期间暂停 read/write）。
- 参数化：`enable_write`、`strict_can`、`damping_kd`。
- README 写清启动顺序、急停方法、常见故障。

## 每步的提交习惯

```bash
git checkout -b step/02-hardware-readonly
# ... 实现并验证 ...
git commit -m "step-02(hardware): read-only CAN path"
git tag step-02-passed
```

`main` 只保留已验证的检查点，出问题随时回退。
