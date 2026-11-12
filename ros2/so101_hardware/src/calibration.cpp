#include "so101_hardware/calibration.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace so101_hardware
{

std::string expand_user(const std::string & path)
{
  if (path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

Calibration parse_calibration(const std::string & json_text)
{
  nlohmann::json doc;
  try {
    doc = nlohmann::json::parse(json_text);
  } catch (const nlohmann::json::parse_error & e) {
    throw std::runtime_error(std::string("calibration: invalid JSON: ") + e.what());
  }
  if (!doc.is_object()) {
    throw std::runtime_error("calibration: top level must be an object of motors");
  }

  Calibration out;
  for (const auto & item : doc.items()) {
    const std::string name = item.key();
    const auto & motor = item.value();
    auto field = [&name, &motor](const char * key) -> int {
        if (!motor.contains(key) || !motor[key].is_number_integer()) {
          throw std::runtime_error(
                  "calibration: motor '" + name + "' is missing integer field '" + key + "'");
        }
        return motor[key].get<int>();
      };
    MotorCalibration m;
    m.id = field("id");
    m.drive_mode = field("drive_mode");
    m.homing_offset = field("homing_offset");
    m.range_min = field("range_min");
    m.range_max = field("range_max");
    if (m.id < 0 || m.id > 253) {
      throw std::runtime_error("calibration: motor '" + name + "' has invalid id");
    }
    // LeRobot only writes 0 (normal) or 1 (inverted). Anything else is a
    // corrupt or hand-edited file, and guessing the direction is unsafe.
    if (m.drive_mode != 0 && m.drive_mode != 1) {
      throw std::runtime_error(
              "calibration: motor '" + name + "' has drive_mode " +
              std::to_string(m.drive_mode) + " (expected 0 or 1)");
    }
    // HOMING_OFFSET is sign-magnitude with the sign in bit 11, so no servo
    // can hold (or report) a larger magnitude.
    if (m.homing_offset < -2047 || m.homing_offset > 2047) {
      throw std::runtime_error(
              "calibration: motor '" + name + "' has homing_offset " +
              std::to_string(m.homing_offset) + " outside -2047..2047");
    }
    if (m.range_min >= m.range_max) {
      throw std::runtime_error(
              "calibration: motor '" + name + "' has range_min >= range_max");
    }
    if (m.range_min < 0 || m.range_max > 4095) {
      throw std::runtime_error(
              "calibration: motor '" + name + "' range is outside 0..4095");
    }
    out.emplace(name, m);
  }
  return out;
}

Calibration load_calibration(const std::string & path)
{
  const std::string resolved = expand_user(path);
  std::ifstream in(resolved);
  if (!in) {
    throw std::runtime_error("calibration: cannot open " + resolved);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return parse_calibration(ss.str());
}

}  // namespace so101_hardware
