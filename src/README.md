# 包清单

| 包 | 一句话 | 自带 / 自写 |
|---|---|---|
| `ros_control_msgs` | 消息 + 服务 | 自写 |
| `ros_control_hardware` | SystemInterface + CAN/电机/运动学 + Mock | 自写（基类自带） |
| `ros_control_controllers` | MIT 关节控制器 | 自写（基类自带） |
| `ros_control_description` | URDF + `<ros2_control>` | 自写（publisher 自带） |
| `ros_control_inference` | ONNX 推理节点 | 自写 |
| `ros_control_imu` | IMU 串口节点 | 自写 |
| `ros_control_joy` | 手柄映射节点 | 自写 |
| `ros_control_bringup` | launch + 参数 | 自写（launch 自带） |
| `ros_control_sdk` | C++ SDK + CLI | 自写 |
| `ros_control_sdk_py` | Python SDK | 自写 |

每个目录目前是空的（只有 `.gitkeep`）。
