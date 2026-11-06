// Reader for LeRobot motor calibration files, e.g.
//   ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json
//
// Reusing LeRobot's calibration means an arm calibrated once with
// `lerobot-calibrate` reports the same joint values in ROS 2 and in LeRobot.
#pragma once

#include <map>
#include <string>

namespace so101_hardware
{

struct MotorCalibration
{
  int id{0};
  int drive_mode{0};      // 1 = direction inverted
  int homing_offset{0};   // value LeRobot wrote to the servo's HOMING_OFFSET
  int range_min{0};       // raw encoder steps (after homing offset)
  int range_max{4095};
};

using Calibration = std::map<std::string, MotorCalibration>;

/// Parse calibration JSON text. Throws std::runtime_error with a message
/// naming the offending motor/field on malformed input.
Calibration parse_calibration(const std::string & json_text);

/// Load and parse a calibration file. A leading "~/" expands to $HOME.
Calibration load_calibration(const std::string & path);

/// Expand a leading "~/" using $HOME.
std::string expand_user(const std::string & path);

}  // namespace so101_hardware
