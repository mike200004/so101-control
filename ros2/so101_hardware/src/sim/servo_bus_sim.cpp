#include "so101_hardware/sim/servo_bus_sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "so101_hardware/feetech/sts3215.hpp"

namespace so101_hardware::sim
{

namespace reg = feetech::sts3215;
using feetech::Instruction;

namespace
{
void put_u16(std::array<uint8_t, 256> & mem, uint8_t address, uint16_t v)
{
  mem[address] = static_cast<uint8_t>(v & 0xFF);
  mem[address + 1] = static_cast<uint8_t>(v >> 8);
}
uint16_t get_u16(const std::array<uint8_t, 256> & mem, uint8_t address)
{
  return static_cast<uint16_t>(mem[address] | (mem[address + 1] << 8));
}

SimServo make_servo(uint8_t id)
{
  SimServo s;
  put_u16(s.mem, reg::MODEL_NUMBER, reg::kModelNumber);
  s.mem[reg::ID] = id;
  s.mem[reg::BAUD_RATE] = 0;  // 1 Mbaud
  s.mem[reg::RETURN_DELAY_TIME] = 250;
  put_u16(s.mem, reg::MAX_POSITION_LIMIT, 4095);
  s.mem[reg::P_COEFFICIENT] = 32;
  s.mem[reg::D_COEFFICIENT] = 32;
  s.mem[reg::LOCK] = 1;
  s.mem[reg::PRESENT_VOLTAGE] = 120;
  s.mem[reg::PRESENT_TEMPERATURE] = 30;
  return s;
}
}  // namespace

ServoBusSim::ServoBusSim(const std::vector<uint8_t> & ids)
{
  for (auto id : ids) {
    servos_.emplace(id, make_servo(id));
    refresh_present(servos_.at(id));
  }
}

ServoBusSim::ServoBusSim(const Calibration & calibration)
{
  for (const auto & [name, cal] : calibration) {
    (void)name;
    const auto id = static_cast<uint8_t>(cal.id);
    SimServo s = make_servo(id);
    put_u16(
      s.mem, reg::HOMING_OFFSET,
      feetech::encode_sign_magnitude(cal.homing_offset, reg::kHomingOffsetSignBit));
    put_u16(s.mem, reg::MIN_POSITION_LIMIT, static_cast<uint16_t>(cal.range_min));
    put_u16(s.mem, reg::MAX_POSITION_LIMIT, static_cast<uint16_t>(cal.range_max));
    s.position = 0.5 * (cal.range_min + cal.range_max);
    refresh_present(s);
    servos_.emplace(id, s);
  }
}

void ServoBusSim::refresh_present(SimServo & s)
{
  const int raw = static_cast<int>(std::lround(s.position));
  put_u16(s.mem, reg::PRESENT_POSITION, feetech::encode_sign_magnitude(raw, reg::kPositionSignBit));
}

std::vector<uint8_t> ServoBusSim::handle(const uint8_t * data, std::size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> out;
  for (std::size_t i = 0; i < size; ++i) {
    if (auto pkt = parser_.push(data[i])) {
      execute(*pkt, out);
    }
  }
  return out;
}

void ServoBusSim::reply(uint8_t id, const std::vector<uint8_t> & params, std::vector<uint8_t> & out)
{
  if (silent_.count(id)) {
    return;
  }
  const uint8_t err = status_error_.count(id) ? status_error_.at(id) : 0;
  // A status packet has the same layout as an instruction, with the error
  // byte in the instruction slot.
  auto pkt = feetech::make_instruction(id, static_cast<Instruction>(err), params);
  out.insert(out.end(), pkt.begin(), pkt.end());
}

void ServoBusSim::write_mem(
  SimServo & s, uint8_t id, uint8_t address, const uint8_t * data, std::size_t n)
{
  for (std::size_t i = 0; i < n && address + i < s.mem.size(); ++i) {
    const auto a = static_cast<uint8_t>(address + i);
    // Read-only status area.
    if (a >= reg::PRESENT_POSITION && a < reg::PRESENT_POSITION + 16) {
      continue;
    }
    if (a < reg::TORQUE_ENABLE) {
      // EEPROM area: honoured only when unlocked, like the real servo.
      if (s.mem[reg::LOCK] != 0) {
        continue;
      }
      ++eeprom_writes_[id];
    }
    s.mem[a] = data[i];
  }
}

void ServoBusSim::execute(const feetech::PacketParser::Packet & pkt, std::vector<uint8_t> & out)
{
  const auto instr = static_cast<Instruction>(pkt.code);
  const auto & p = pkt.params;

  if (instr == Instruction::kSyncWrite) {
    if (p.size() < 2) {
      return;
    }
    const uint8_t address = p[0];
    const uint8_t len = p[1];
    for (std::size_t off = 2; off + 1 + len <= p.size(); off += 1 + len) {
      auto it = servos_.find(p[off]);
      if (it != servos_.end()) {
        write_mem(it->second, it->first, address, &p[off + 1], len);
      }
    }
    return;  // sync write never replies
  }

  if (instr == Instruction::kSyncRead) {
    if (p.size() < 2) {
      return;
    }
    const uint8_t address = p[0];
    const uint8_t len = p[1];
    if (address + len > 256) {
      return;
    }
    for (std::size_t i = 2; i < p.size(); ++i) {
      auto it = servos_.find(p[i]);
      if (it == servos_.end()) {
        continue;
      }
      std::vector<uint8_t> data(
        it->second.mem.begin() + address, it->second.mem.begin() + address + len);
      reply(it->first, data, out);
    }
    return;
  }

  auto it = servos_.find(pkt.id);
  if (it == servos_.end()) {
    return;  // nobody on the bus with that ID
  }
  SimServo & s = it->second;

  switch (instr) {
    case Instruction::kPing:
      reply(pkt.id, {}, out);
      break;
    case Instruction::kRead:
      if (p.size() == 2 && p[0] + p[1] <= 256) {
        reply(pkt.id, std::vector<uint8_t>(s.mem.begin() + p[0], s.mem.begin() + p[0] + p[1]), out);
      }
      break;
    case Instruction::kWrite:
      if (!p.empty()) {
        write_mem(s, pkt.id, p[0], p.data() + 1, p.size() - 1);
        reply(pkt.id, {}, out);
      }
      break;
    default:
      break;
  }
}

void ServoBusSim::step(double dt_s, double max_speed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [id, s] : servos_) {
    (void)id;
    double velocity = 0.0;
    if (s.mem[reg::TORQUE_ENABLE]) {
      const double goal = feetech::decode_sign_magnitude(
        get_u16(s.mem, reg::GOAL_POSITION), reg::kPositionSignBit);
      const double err = goal - s.position;
      const double max_step = max_speed * dt_s;
      const double delta = std::clamp(err, -max_step, max_step);
      s.position += delta;
      velocity = dt_s > 0 ? delta / dt_s : 0.0;
    }
    refresh_present(s);
    const int v = static_cast<int>(std::lround(std::clamp(velocity, -32767.0, 32767.0)));
    put_u16(s.mem, reg::PRESENT_VELOCITY, feetech::encode_sign_magnitude(v, reg::kVelocitySignBit));
  }
}

uint16_t ServoBusSim::reg_u16(uint8_t id, uint8_t address) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return get_u16(servos_.at(id).mem, address);
}

uint8_t ServoBusSim::reg_u8(uint8_t id, uint8_t address) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return servos_.at(id).mem[address];
}

void ServoBusSim::set_reg_u16(uint8_t id, uint8_t address, uint16_t value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  put_u16(servos_.at(id).mem, address, value);
}

void ServoBusSim::set_reg_u8(uint8_t id, uint8_t address, uint8_t value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  servos_.at(id).mem[address] = value;
}

void ServoBusSim::set_position(uint8_t id, double raw)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & s = servos_.at(id);
  s.position = raw;
  refresh_present(s);
}

void ServoBusSim::set_silent(uint8_t id, bool silent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (silent) {
    silent_.insert(id);
  } else {
    silent_.erase(id);
  }
}

void ServoBusSim::set_status_error(uint8_t id, uint8_t error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_error_[id] = error;
}

int ServoBusSim::eeprom_writes(uint8_t id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = eeprom_writes_.find(id);
  return it == eeprom_writes_.end() ? 0 : it->second;
}

// ---------------------------------------------------------------------------

void SimTransport::write(const uint8_t * data, std::size_t size)
{
  tx_log_.insert(tx_log_.end(), data, data + size);
  const auto reply = sim_.handle(data, size);
  rx_.insert(rx_.end(), reply.begin(), reply.end());
}

std::size_t SimTransport::read(uint8_t * data, std::size_t size, std::chrono::microseconds)
{
  const std::size_t n = std::min(size, rx_.size());
  for (std::size_t i = 0; i < n; ++i) {
    data[i] = rx_.front();
    rx_.pop_front();
  }
  return n;
}

}  // namespace so101_hardware::sim
