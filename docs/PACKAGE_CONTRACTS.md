# 每包契约：建哪些文件、对外什么接口、怎么验收

> 只列"必须存在"的东西，内部实现自由。

## 1. ros_control_msgs（ament_cmake）

```
package.xml
CMakeLists.txt                 # rosidl_generate_interfaces(...)
msg/MitJointCommand.msg
msg/MotorState.msg
msg/MotorStateArray.msg
msg/InferenceStatus.msg
srv/SwitchMode.srv
srv/SetZeroPosition.srv
```

`MitJointCommand`：`std_msgs/Header header` + `string[] name` + `float64[] position/velocity/effort/kp/kd`，数组等长。

`SwitchMode`：常量 `IDLE=0 CROUCH=1 INFERENCE=2 DAMPING=3`；请求 `uint8 mode`；响应 `bool success, uint8 current_mode, string message`。

`SetZeroPosition`：请求 `string can_bus`；响应 `bool success, uint32 requested, uint32 succeeded, string message`。

验收：

```bash
colcon build --packages-select ros_control_msgs && source install/setup.bash
ros2 interface show ros_control_msgs/msg/MitJointCommand
```

## 2. ros_control_hardware（ament_cmake）

```
package.xml
CMakeLists.txt
ros_control_hardware_plugins.xml        # pluginlib 描述
config/g1_14dof_hardware.yaml           # 22 电机 + CAN + 安全参数
include/<pkg>/hardware_config.hpp       # YAML -> 结构体 + 校验
include/<pkg>/system_hardware.hpp
include/<pkg>/mock_system.hpp           # 无 CAN 调试用
include/<pkg>/hardware_admin.hpp        # 电机遥测 + 置零服务
src/*.cpp
```

插件类：

- `ros_control_hardware/SystemHardware` → `hardware_interface::SystemInterface`
- `ros_control_hardware/MockSystem` → 同一基类，位置直接跟随指令

约定：

- 22 个电机，顺序 = 物理接线（0-9 手臂，10-15 左腿，16-21 右腿），运动学矩阵按固定偏移取数。
- 14 个控制关节通过 `motors[].joint` 名字关联，不靠下标。
- 无 `joint:` 字段的电机 = 被动电机（手臂 8 轴），位置模式回零后不参与 MIT。

验收（mock）：`ros2 control list_hardware_interfaces` 能看到 14 关节 × (5 command + 3 state)。
验收（真机只读）：`ros2 topic echo /motor_states --once` 22 个电机、online=true。

## 3. ros_control_controllers（ament_cmake）

```
package.xml / CMakeLists.txt / plugins.xml
include/<pkg>/mit_joint_controller.hpp
src/mit_joint_controller.cpp
```

参数：`joints`（14）、`command_interfaces`、`state_interfaces`、`command_topic`（默认 `~/commands`）、`command_timeout`、`damping_kd`、`default_kp/kd`、`max_kp/kd`。

验收：`ros2 control list_controllers` 显示 `mit_joint_controller: active`。

## 4. ros_control_description（ament_cmake）

```
package.xml / CMakeLists.txt
urdf/g1_14dof.urdf.xacro
```

参数：`mock`、`enable_write`、`hardware_config`。

真实限位（来自训练用 MJCF，别凭感觉写，否则策略目标会被 clamp）：

| 关节 | lower | upper |
|---|---|---|
| hip_yaw（左/右） | -0.518 / -0.785 | 0.518 / 0.785 |
| hip_roll（左/右） | -0.175 / -0.96 | 0.96 / 0.175 |
| hip_pitch | -1.047 | 0.524 |
| knee | -0.087 | 1.745 |
| ankle_pitch | -1.082 | 1.082 |
| ankle_roll | -0.664 | 0.664 |
| shoulder_pitch | -3.1416 | 1.5708 |

验收：

```bash
xacro urdf/g1_14dof.urdf.xacro mock:=true > /tmp/r.urdf && grep -c command_interface /tmp/r.urdf
# 期望 70（14 × 5）
```

## 5. ros_control_inference（ament_cmake）

```
package.xml / CMakeLists.txt / config/policy.yaml
include/<pkg>/policy.hpp        # ONNX Runtime 封装
include/<pkg>/infer_node.hpp
src/policy.cpp / infer_node.cpp / main.cpp
```

观测布局（必须和训练一致，255 = 51 × 5）：

```
[ang_vel(3), projected_gravity(3), pos-default(14), vel(14), cmd(3), tracking_error(14)]
× 5 帧，按"项优先、旧→新"拼接
```

控制：50Hz 推理、200Hz 下发；`target = default + action * joint_scale`，目标一阶低通（α≈0.2）。

验收：mock 下 `ros2 topic hz /mit_joint_controller/commands` ≈ 200，`/joint_states` 平滑。

## 6. ros_control_imu（ament_cmake）

`imu_data.hpp / imu_driver.hpp / decoders / imu_node.cpp`，发布 `sensor_msgs/Imu`（orientation + angular_velocity + linear_acceleration），可选中值/低通滤波。

验收：`ros2 topic hz /imu` ≈ 100。

## 7. ros_control_joy（ament_cmake）

订阅 `/joy`，发布 `/cmd_vel`（Twist）和 `/control_mode`（UInt8）。

按键：LB 使能速度、A=CROUCH、X=INFERENCE、B=IDLE。

验收：按 X 后 `/control_mode` 变 2。

## 8. ros_control_bringup（ament_cmake）

```
config/controllers.yaml
launch/mock.launch.py
launch/robot.launch.py
```

`mock.launch.py`：robot_state_publisher + ros2_control_node + spawner（joint_state_broadcaster、mit_joint_controller）。
`robot.launch.py`：同上，`mock:=false`，默认 `enable_write:=false`。

验收：

```bash
ros2 launch ros_control_bringup mock.launch.py
ros2 topic hz /joint_states        # ≈200Hz
```

## 9. ros_control_sdk / ros_control_sdk_py

C++：`include/ros_control_sdk/robot.hpp` + `src/robot.cpp` + `src/cli.cpp`，导出 CMake target。
Python：`ros_control_sdk/__init__.py`、`robot.py`、`__main__.py`。

最小 API：

```text
Robot(node_or_options)
  wait_for_state(timeout)
  get_joint_state()
  set_velocity(vx, vy, wz)
  set_mode(mode)        # IDLE/CROUCH/INFERENCE/DAMPING
  start_inference() / stop() / crouch() / damp()
  set_zero_position(bus)
  send_joint_targets(position, kp, kd)
```

验收：`ros2 run ros_control_sdk ros_control_cli status`。
