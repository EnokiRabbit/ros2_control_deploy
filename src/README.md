# 包清单

| 包 | 一句话 | 契约 |
|---|---|---|
| `ros_control_msgs` | 消息/服务定义 | PACKAGE_CONTRACTS §1 |
| `ros_control_hardware` | ros2_control SystemInterface（CAN/电机/运动学）+ Mock | §2 |
| `ros_control_controllers` | `mit_joint_controller` | §3 |
| `ros_control_description` | URDF/xacro + `<ros2_control>` | §4 |
| `ros_control_inference` | ONNX 推理节点 | §5 |
| `ros_control_imu` | IMU 节点 | §6 |
| `ros_control_joy` | 手柄节点 | §7 |
| `ros_control_bringup` | launch + 参数 | §8 |
| `ros_control_sdk` | C++ SDK + CLI | §9 |
| `ros_control_sdk_py` | Python SDK | §9 |

每个目录目前是空的（只有 `.gitkeep`），按 `docs/IMPLEMENTATION.md` 的顺序填充。
