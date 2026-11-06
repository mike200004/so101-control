// A software model of a chain of STS3215 servos that speaks the real wire
// protocol. It is used
//   * in-process (SimTransport) by unit tests of the protocol and the
//     ros2_control plugin, and
//   * over a pseudo-terminal by the `fake_servo_bus` executable, so the full
//     ROS 2 stack (serial port included) can run in CI without an arm.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <vector>

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech/protocol.hpp"
#include "so101_hardware/feetech/transport.hpp"

namespace so101_hardware::sim
{

struct SimServo
{
  std::array<uint8_t, 256> mem{};
  double position{2048.0};  // continuous internal state, raw steps
};

class ServoBusSim
{
public:
  /// Creates servos with the given IDs, all at raw position 2048.
  explicit ServoBusSim(const std::vector<uint8_t> & ids);
  /// Creates one servo per calibration entry, homing offset preloaded and
  /// parked in the middle of its calibrated range.
  explicit ServoBusSim(const Calibration & calibration);

  /// Feed bytes written by the host; returns bytes the servos reply with.
  std::vector<uint8_t> handle(const uint8_t * data, std::size_t size);

  /// Advance the motion model: torqued servos move toward their goal at
  /// `max_speed` steps/s.
  void step(double dt_s, double max_speed = 4000.0);

  // --- inspection / fault injection (thread-safe) --------------------------
  uint16_t reg_u16(uint8_t id, uint8_t address) const;
  uint8_t reg_u8(uint8_t id, uint8_t address) const;
  void set_reg_u16(uint8_t id, uint8_t address, uint16_t value);
  void set_reg_u8(uint8_t id, uint8_t address, uint8_t value);
  void set_position(uint8_t id, double raw);
  /// Servos in this set stop replying (simulates a loose cable).
  void set_silent(uint8_t id, bool silent);
  /// Error byte reported in every status packet from `id`.
  void set_status_error(uint8_t id, uint8_t error);
  /// Count of EEPROM-area writes (address < 40) received per servo.
  int eeprom_writes(uint8_t id) const;

private:
  void execute(const feetech::PacketParser::Packet & pkt, std::vector<uint8_t> & out);
  void reply(uint8_t id, const std::vector<uint8_t> & params, std::vector<uint8_t> & out);
  void write_mem(SimServo & s, uint8_t id, uint8_t address, const uint8_t * data, std::size_t n);
  void refresh_present(SimServo & s);

  mutable std::mutex mutex_;
  std::map<uint8_t, SimServo> servos_;
  std::set<uint8_t> silent_;
  std::map<uint8_t, uint8_t> status_error_;
  std::map<uint8_t, int> eeprom_writes_;
  feetech::PacketParser parser_;
};

/// In-process Transport backed by a ServoBusSim.
class SimTransport final : public feetech::Transport
{
public:
  explicit SimTransport(ServoBusSim & sim) : sim_(sim) {}
  void write(const uint8_t * data, std::size_t size) override;
  std::size_t read(uint8_t * data, std::size_t size, std::chrono::microseconds timeout) override;
  void flush_input() override { rx_.clear(); }
  /// Every byte the host has written (for asserting on wire traffic).
  const std::vector<uint8_t> & tx_log() const { return tx_log_; }
  void clear_tx_log() { tx_log_.clear(); }

private:
  ServoBusSim & sim_;
  std::deque<uint8_t> rx_;
  std::vector<uint8_t> tx_log_;
};

}  // namespace so101_hardware::sim
