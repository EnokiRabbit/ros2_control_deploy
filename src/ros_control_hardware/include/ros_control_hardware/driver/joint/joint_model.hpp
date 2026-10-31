#pragma once

namespace ros_control_hardware {

/// 设备单位 <-> 关节单位
///
///   joint  = direction * device / gear_ratio + offset
///   device = direction * (joint - offset) * gear_ratio
class JointModel {
public:
  JointModel() = default;
  JointModel(double direction, double gear_ratio, double offset = 0.0)
      : direction_{direction}, gear_ratio_{gear_ratio}, offset_{offset} {}

  [[nodiscard]] double toJoint(double device_value) const
  {
    return direction_ * device_value / gear_ratio_ + offset_;
  }

  [[nodiscard]] double toDevice(double joint_value) const
  {
    return direction_ * (joint_value - offset_) * gear_ratio_;
  }

  [[nodiscard]] double direction() const { return direction_; }
  [[nodiscard]] double gearRatio() const { return gear_ratio_; }
  [[nodiscard]] double offset() const { return offset_; }

private:
  double direction_{1.0};
  double gear_ratio_{1.0};
  double offset_{0.0};
};

}  // namespace ros_control_hardware
