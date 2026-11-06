#include "so101_hardware/joint_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "so101_hardware/feetech/sts3215.hpp"

namespace so101_hardware
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;
}

MappingKind parse_mapping_kind(const std::string & s)
{
  if (s == "revolute") {
    return MappingKind::kRevolute;
  }
  if (s == "linear") {
    return MappingKind::kLinear;
  }
  throw std::invalid_argument("unknown joint mapping '" + s + "' (expected revolute|linear)");
}

JointMapping::JointMapping(
  MappingKind kind, const MotorCalibration & cal, double lower, double upper)
: kind_(kind), cal_(cal), lower_(lower), upper_(upper)
{
  if (cal.range_min >= cal.range_max) {
    throw std::invalid_argument("joint mapping: empty calibrated range");
  }
  sign_ = cal.drive_mode ? -1.0 : 1.0;
  if (kind_ == MappingKind::kRevolute) {
    mid_ = 0.5 * (cal.range_min + cal.range_max);
    scale_ = kTwoPi / feetech::sts3215::kResolution;
  } else {
    if (!std::isfinite(lower) || !std::isfinite(upper) || !(upper > lower)) {
      throw std::invalid_argument(
              "joint mapping: linear mapping needs finite limits with upper > lower "
              "(set min/max on the joint's position command interface)");
    }
    mid_ = cal.range_min;
    scale_ = (upper - lower) / (cal.range_max - cal.range_min);
  }
}

double JointMapping::raw_to_position(int raw) const
{
  if (kind_ == MappingKind::kRevolute) {
    return sign_ * (raw - mid_) * scale_;
  }
  // Linear: range_min -> lower, range_max -> upper (reversed if drive_mode).
  const double t = (raw - mid_) * scale_;
  return sign_ > 0 ? lower_ + t : upper_ - t;
}

int JointMapping::position_to_raw(double position) const
{
  double raw;
  if (kind_ == MappingKind::kRevolute) {
    raw = mid_ + sign_ * position / scale_;
  } else {
    const double t = sign_ > 0 ? position - lower_ : upper_ - position;
    raw = mid_ + t / scale_;
  }
  if (!std::isfinite(raw)) {
    throw std::invalid_argument("joint mapping: non-finite position command");
  }
  const long r = std::lround(raw);
  return static_cast<int>(std::clamp<long>(r, cal_.range_min, cal_.range_max));
}

double JointMapping::raw_to_velocity(int raw_steps_per_s) const
{
  return sign_ * raw_steps_per_s * scale_;
}

}  // namespace so101_hardware
