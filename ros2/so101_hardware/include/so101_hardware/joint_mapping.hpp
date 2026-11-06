// Conversion between raw STS3215 encoder steps and URDF joint units.
//
// Arm joints ("revolute"): the joint zero is the middle of the calibrated
// range, which is both LeRobot's `MotorNormMode.DEGREES` convention and the
// zero pose of the upstream `so101_new_calib.urdf`:
//
//     q [rad] = s * (raw - (range_min + range_max) / 2) * 2*pi / 4096
//
// where s = -1 when drive_mode == 1, else +1.
//
// Gripper ("linear"): LeRobot exposes the gripper as 0..100 % of its
// calibrated range. ROS models it as a revolute jaw, so the calibrated range
// is mapped linearly onto the URDF joint limits [lower, upper].
#pragma once

#include <cstdint>
#include <string>

#include "so101_hardware/calibration.hpp"

namespace so101_hardware
{

enum class MappingKind { kRevolute, kLinear };

MappingKind parse_mapping_kind(const std::string & s);

class JointMapping
{
public:
  JointMapping() = default;
  /// `lower`/`upper` are the URDF limits; required for kLinear, used for
  /// command clamping in both modes.
  JointMapping(MappingKind kind, const MotorCalibration & cal, double lower, double upper);

  double raw_to_position(int raw) const;
  /// Converts and clamps the result to the calibrated raw range.
  int position_to_raw(double position) const;
  /// Velocity register (steps/s) to rad/s. For the linear gripper mapping this
  /// is scaled by the same factor as position.
  double raw_to_velocity(int raw_steps_per_s) const;

  MappingKind kind() const { return kind_; }
  const MotorCalibration & calibration() const { return cal_; }

private:
  MappingKind kind_{MappingKind::kRevolute};
  MotorCalibration cal_;
  double lower_{0.0};
  double upper_{0.0};
  double sign_{1.0};
  double mid_{2047.5};
  double scale_{0.0};  // rad per step
};

}  // namespace so101_hardware
