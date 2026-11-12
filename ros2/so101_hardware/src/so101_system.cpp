#include "so101_hardware/so101_system.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech/serial_port.hpp"
#include "so101_hardware/feetech/sts3215.hpp"

namespace so101_hardware
{

namespace reg = feetech::sts3215;
using hardware_interface::return_type;

namespace
{

std::string get_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

bool parse_bool(const std::string & s)
{
  if (s == "true" || s == "True" || s == "1") {
    return true;
  }
  if (s == "false" || s == "False" || s == "0") {
    return false;
  }
  throw std::invalid_argument("expected boolean, got '" + s + "'");
}

double parse_limit(const std::string & s, double fallback)
{
  return s.empty() ? fallback : std::stod(s);
}

void require_at_least(const std::string & name, int value, int minimum)
{
  if (value < minimum) {
    throw std::invalid_argument(
            name + " must be at least " + std::to_string(minimum) + ", got " +
            std::to_string(value));
  }
}

void require_gain(const std::string & name, int value)
{
  // Gains are 8-bit EEPROM registers; a negative value means "leave as is".
  if (value > 255) {
    throw std::invalid_argument(
            name + " must be 0..255 (or negative to leave the servo's value unchanged), got " +
            std::to_string(value));
  }
}

}  // namespace

SO101SystemHardware::SO101SystemHardware()
: logger_(rclcpp::get_logger("SO101SystemHardware"))
{
  transport_factory_ = [](const HardwareParams & p) -> std::unique_ptr<feetech::Transport> {
      return std::make_unique<feetech::SerialPort>(p.port, p.baudrate);
    };
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    const auto & hp = info_.hardware_parameters;
    params_.port = get_param(hp, "port", params_.port);
    params_.baudrate = std::stoi(get_param(hp, "baudrate", std::to_string(params_.baudrate)));
    params_.calibration_file = get_param(hp, "calibration_file", "");
    params_.timeout_us = std::stoi(get_param(hp, "timeout_us", std::to_string(params_.timeout_us)));
    params_.verify_homing_offset = parse_bool(get_param(hp, "verify_homing_offset", "true"));
    params_.torque_off_on_deactivate =
      parse_bool(get_param(hp, "torque_off_on_deactivate", "true"));
    params_.max_consecutive_read_failures = std::stoi(
      get_param(
        hp, "max_consecutive_read_failures",
        std::to_string(params_.max_consecutive_read_failures)));
    params_.p_coefficient = std::stoi(get_param(hp, "p_coefficient", "16"));
    params_.i_coefficient = std::stoi(get_param(hp, "i_coefficient", "0"));
    params_.d_coefficient = std::stoi(get_param(hp, "d_coefficient", "32"));

    require_at_least("baudrate", params_.baudrate, 1);
    require_at_least("timeout_us", params_.timeout_us, 1);
    // 0 would make read() fail on the first missed reply instead of tolerating it.
    require_at_least("max_consecutive_read_failures", params_.max_consecutive_read_failures, 1);
    require_gain("p_coefficient", params_.p_coefficient);
    require_gain("i_coefficient", params_.i_coefficient);
    require_gain("d_coefficient", params_.d_coefficient);

    if (params_.calibration_file.empty()) {
      RCLCPP_FATAL(
        logger_,
        "Parameter 'calibration_file' is required (a LeRobot calibration JSON, e.g. "
        "~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json)");
      return CallbackReturn::ERROR;
    }
    const Calibration calibration = load_calibration(params_.calibration_file);

    joints_.clear();
    ids_.clear();
    std::set<int> seen_ids;
    for (const auto & ji : info_.joints) {
      if (ji.command_interfaces.size() != 1 ||
        ji.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(logger_, "Joint '%s' must have exactly one 'position' command interface",
          ji.name.c_str());
        return CallbackReturn::ERROR;
      }
      for (const auto & si : ji.state_interfaces) {
        if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(logger_, "Joint '%s': unsupported state interface '%s'",
            ji.name.c_str(), si.name.c_str());
          return CallbackReturn::ERROR;
        }
      }

      const std::string motor_name = get_param(ji.parameters, "motor_name", ji.name);
      auto cal_it = calibration.find(motor_name);
      if (cal_it == calibration.end()) {
        RCLCPP_FATAL(logger_, "Calibration file has no entry for motor '%s'", motor_name.c_str());
        return CallbackReturn::ERROR;
      }
      MotorCalibration cal = cal_it->second;
      if (ji.parameters.count("id")) {
        cal.id = std::stoi(ji.parameters.at("id"));
        // Same range the calibration parser enforces; anything else would
        // silently wrap in the 8-bit ID field and address a different servo.
        if (cal.id < 0 || cal.id > 253) {
          throw std::invalid_argument(
                  "joint '" + ji.name + "': servo id " + std::to_string(cal.id) +
                  " is outside 0..253");
        }
      }
      if (!seen_ids.insert(cal.id).second) {
        RCLCPP_FATAL(logger_, "Servo ID %d is assigned to more than one joint", cal.id);
        return CallbackReturn::ERROR;
      }

      const auto kind = parse_mapping_kind(get_param(ji.parameters, "mapping", "revolute"));
      const auto & ci = ji.command_interfaces[0];
      const double lower = parse_limit(ci.min, -std::numeric_limits<double>::infinity());
      const double upper = parse_limit(ci.max, std::numeric_limits<double>::infinity());

      Joint j;
      j.name = ji.name;
      j.id = static_cast<uint8_t>(cal.id);
      j.mapping = JointMapping(kind, cal, lower, upper);
      j.command = std::numeric_limits<double>::quiet_NaN();
      joints_.push_back(std::move(j));
      ids_.push_back(static_cast<uint8_t>(cal.id));
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Invalid configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Configured %zu joints on %s @ %d baud (calibration: %s)", joints_.size(),
    params_.port.c_str(), params_.baudrate, params_.calibration_file.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      double * value = si.name == hardware_interface::HW_IF_POSITION ?
        &joints_[i].position : &joints_[i].velocity;
      out.emplace_back(joints_[i].name, si.name, value);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> SO101SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.command);
  }
  return out;
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    transport_ = transport_factory_(params_);
    bus_ = std::make_unique<feetech::Bus>(
      *transport_, std::chrono::microseconds(params_.timeout_us));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Cannot open servo bus: %s", e.what());
    close_bus();
    return CallbackReturn::ERROR;
  }

  try {
    for (auto & j : joints_) {
      // Ping twice: the first packet after opening a port is sometimes lost
      // while the USB bridge settles.
      if (!bus_->ping(j.id) && !bus_->ping(j.id)) {
        RCLCPP_ERROR(
          logger_, "Servo %d (%s) does not answer on %s. Check power, cabling and IDs.",
          j.id, j.name.c_str(), params_.port.c_str());
        close_bus();
        return CallbackReturn::ERROR;
      }
      const uint16_t model = bus_->read_u16(j.id, reg::MODEL_NUMBER);
      if (model != reg::kModelNumber) {
        RCLCPP_ERROR(
          logger_, "Servo %d (%s) reports model %u, expected STS3215 (%u)", j.id,
          j.name.c_str(), model, reg::kModelNumber);
        close_bus();
        return CallbackReturn::ERROR;
      }
      if (params_.verify_homing_offset) {
        const int offset = feetech::decode_sign_magnitude(
          bus_->read_u16(j.id, reg::HOMING_OFFSET), reg::kHomingOffsetSignBit);
        const int expected = j.mapping.calibration().homing_offset;
        if (offset != expected) {
          RCLCPP_ERROR(
            logger_,
            "Servo %d (%s) has homing offset %d but the calibration file says %d. "
            "This usually means the wrong arm/port or a stale calibration file; "
            "joint angles would be wrong. Recalibrate with lerobot-calibrate or set "
            "verify_homing_offset:=false if you know what you are doing.",
            j.id, j.name.c_str(), offset, expected);
          close_bus();
          return CallbackReturn::ERROR;
        }
      }
    }
    if (!read_all_positions_blocking()) {
      close_bus();
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Servo bus error during configure: %s", e.what());
    close_bus();
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "All %zu servos found and verified", joints_.size());
  return CallbackReturn::SUCCESS;
}

bool SO101SystemHardware::read_all_positions_blocking()
{
  for (auto & j : joints_) {
    int raw = 0;
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      try {
        raw = feetech::decode_sign_magnitude(
          bus_->read_u16(j.id, reg::PRESENT_POSITION), reg::kPositionSignBit);
        ok = true;
      } catch (const feetech::CommError &) {
      }
    }
    if (!ok) {
      RCLCPP_ERROR(logger_, "Cannot read position of servo %d (%s)", j.id, j.name.c_str());
      return false;
    }
    j.position = j.mapping.raw_to_position(raw);
    j.velocity = 0.0;
  }
  return true;
}

void SO101SystemHardware::apply_gains(Joint & j)
{
  const std::pair<uint8_t, int> gains[] = {
    {reg::P_COEFFICIENT, params_.p_coefficient},
    {reg::I_COEFFICIENT, params_.i_coefficient},
    {reg::D_COEFFICIENT, params_.d_coefficient},
  };
  bool unlocked = false;
  for (const auto & [address, value] : gains) {
    if (value < 0) {
      continue;
    }
    // Gains live in EEPROM: only write when they differ, to avoid wearing it.
    if (bus_->read_u8(j.id, address) == static_cast<uint8_t>(value)) {
      continue;
    }
    if (!unlocked) {
      bus_->write_u8(j.id, reg::LOCK, 0);
      unlocked = true;
    }
    bus_->write_u8(j.id, address, static_cast<uint8_t>(value));
  }
  if (unlocked) {
    bus_->write_u8(j.id, reg::LOCK, 1);
  }
}

void SO101SystemHardware::set_torque(bool enable)
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> data;
  for (const auto & j : joints_) {
    data.push_back({j.id, {static_cast<uint8_t>(enable ? 1 : 0)}});
  }
  bus_->sync_write(reg::TORQUE_ENABLE, 1, data);
  torque_enabled_ = enable;
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!bus_) {
    return CallbackReturn::ERROR;
  }
  try {
    if (!read_all_positions_blocking()) {
      return CallbackReturn::ERROR;
    }
    for (auto & j : joints_) {
      if (bus_->read_u8(j.id, reg::OPERATING_MODE) != 0) {
        bus_->write_u8(j.id, reg::LOCK, 0);
        bus_->write_u8(j.id, reg::OPERATING_MODE, 0);
        bus_->write_u8(j.id, reg::LOCK, 1);
      }
      apply_gains(j);
      // Hold the current pose: goal = present before torque comes on, so
      // activating never makes the arm jump to a stale goal.
      j.command = j.position;
      j.last_goal_raw = j.mapping.position_to_raw(j.position);
      bus_->write_u16(
        j.id, reg::GOAL_POSITION,
        feetech::encode_sign_magnitude(j.last_goal_raw, reg::kPositionSignBit));
    }
    set_torque(true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  consecutive_read_failures_ = 0;
  RCLCPP_INFO(logger_, "Torque enabled; holding current pose");
  return CallbackReturn::SUCCESS;
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (bus_ && params_.torque_off_on_deactivate) {
    try {
      set_torque(false);
      RCLCPP_WARN(logger_, "Torque disabled: the arm is no longer holding its pose");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to disable torque: %s", e.what());
    }
  }
  return CallbackReturn::SUCCESS;
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_bus();
  return CallbackReturn::SUCCESS;
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  if (torque_enabled_) {
    on_deactivate(state);
  }
  close_bus();
  return CallbackReturn::SUCCESS;
}

SO101SystemHardware::CallbackReturn SO101SystemHardware::on_error(const rclcpp_lifecycle::State &)
{
  // Best effort: stop driving the motors, then release the port.
  if (bus_ && torque_enabled_) {
    try {
      set_torque(false);
    } catch (...) {
    }
  }
  close_bus();
  return CallbackReturn::SUCCESS;
}

void SO101SystemHardware::close_bus()
{
  bus_.reset();
  transport_.reset();
  torque_enabled_ = false;
}

return_type SO101SystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) {
    return return_type::ERROR;
  }
  std::vector<std::optional<std::vector<uint8_t>>> replies;
  try {
    // Position (56..57) and velocity (58..59) in one 4-byte sync read.
    replies = bus_->sync_read(reg::PRESENT_POSITION, 4, ids_);
  } catch (const feetech::TransportError & e) {
    RCLCPP_ERROR(logger_, "Servo bus transport failure: %s", e.what());
    return return_type::ERROR;
  }

  bool all_ok = true;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    if (!replies[i]) {
      all_ok = false;
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 1000, "No state reply from servo %d (%s)", j.id, j.name.c_str());
      continue;
    }
    const auto & d = *replies[i];
    const int raw_pos =
      feetech::decode_sign_magnitude(feetech::from_u16_le(&d[0]), reg::kPositionSignBit);
    const int raw_vel =
      feetech::decode_sign_magnitude(feetech::from_u16_le(&d[2]), reg::kVelocitySignBit);
    j.position = j.mapping.raw_to_position(raw_pos);
    j.velocity = j.mapping.raw_to_velocity(raw_vel);

    if (const uint8_t err = bus_->last_status_error(j.id)) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000, "Servo %d (%s) reports: %s", j.id, j.name.c_str(),
        feetech::describe_status_error(err).c_str());
    }
  }

  if (all_ok) {
    consecutive_read_failures_ = 0;
    return return_type::OK;
  }
  if (++consecutive_read_failures_ >= params_.max_consecutive_read_failures) {
    RCLCPP_ERROR(
      logger_, "%d consecutive incomplete reads from the servo bus; stopping",
      consecutive_read_failures_);
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type SO101SystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) {
    return return_type::ERROR;
  }
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> goals;
  for (auto & j : joints_) {
    if (!std::isfinite(j.command)) {
      continue;  // no controller has claimed this joint yet
    }
    const int raw = j.mapping.position_to_raw(j.command);
    if (raw == j.last_goal_raw) {
      continue;  // unchanged: keep bus time free for reads
    }
    goals.push_back({j.id, feetech::u16_le(
        feetech::encode_sign_magnitude(raw, reg::kPositionSignBit))});
    j.last_goal_raw = raw;
  }
  try {
    bus_->sync_write(reg::GOAL_POSITION, 2, goals);
  } catch (const feetech::TransportError & e) {
    RCLCPP_ERROR(logger_, "Servo bus transport failure: %s", e.what());
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101SystemHardware, hardware_interface::SystemInterface)
