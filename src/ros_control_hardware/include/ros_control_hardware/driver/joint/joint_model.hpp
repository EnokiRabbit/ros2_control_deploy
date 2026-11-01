#pragma once

namespace ros_control_hardware {

/// Device units <-> joint units.
///
/// Position: joint angle includes the zero offset.
///   joint  = direction * device / gear_ratio + offset
///   device = direction * (joint - offset) * gear_ratio
///
/// Velocity: no zero offset.
///   joint  = direction * device / gear_ratio
///   device = direction * joint * gear_ratio
///
/// Effort: the reducer trades speed for torque, joint side = motor side * gear ratio.
///   joint  = direction * gear_ratio * device
///   device = direction * joint / gear_ratio
class JointModel {
public:
  JointModel() = default;
  JointModel(double direction, double gear_ratio, double offset = 0.0)
      : direction_{direction}, gear_ratio_{gear_ratio}, offset_{offset} {}

  // ---- Position ----
  [[nodiscard]] double toJointPosition(double device_value) const
  {
    return direction_ * device_value / gear_ratio_ + offset_;
  }

  [[nodiscard]] double toDevicePosition(double joint_value) const
  {
    return direction_ * (joint_value - offset_) * gear_ratio_;
  }

  // ---- Velocity ----
  [[nodiscard]] double toJointVelocity(double device_value) const
  {
    return direction_ * device_value / gear_ratio_;
  }

  [[nodiscard]] double toDeviceVelocity(double joint_value) const
  {
    return direction_ * joint_value * gear_ratio_;
  }

  // ---- Effort ----
  [[nodiscard]] double toJointEffort(double device_value) const
  {
    return direction_ * gear_ratio_ * device_value;
  }

  [[nodiscard]] double toDeviceEffort(double joint_value) const
  {
    return direction_ * joint_value / gear_ratio_;
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
