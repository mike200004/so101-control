#include "so101_hardware/feetech/protocol.hpp"

#include <cstdlib>
#include <sstream>

namespace so101_hardware::feetech
{

std::string describe_status_error(uint8_t error)
{
  if (error == 0) {
    return "ok";
  }
  std::string out;
  auto add = [&out](const char * name) {
      if (!out.empty()) {
        out += '|';
      }
      out += name;
    };
  if (error & status_error::kVoltage) {add("voltage");}
  if (error & status_error::kAngle) {add("angle");}
  if (error & status_error::kOverheat) {add("overheat");}
  if (error & status_error::kOverCurrent) {add("overcurrent");}
  if (error & status_error::kOverload) {add("overload");}
  const uint8_t known = status_error::kVoltage | status_error::kAngle | status_error::kOverheat |
    status_error::kOverCurrent | status_error::kOverload;
  if (error & ~known) {
    std::ostringstream ss;
    ss << "unknown(0x" << std::hex << static_cast<int>(error & ~known) << ")";
    add(ss.str().c_str());
  }
  return out;
}

uint8_t checksum(const uint8_t * begin, const uint8_t * end)
{
  unsigned sum = 0;
  for (const uint8_t * p = begin; p != end; ++p) {
    sum += *p;
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

std::vector<uint8_t> make_instruction(
  uint8_t id, Instruction instruction, const std::vector<uint8_t> & params)
{
  if (params.size() > 253) {
    throw std::invalid_argument("feetech: too many parameters for one packet");
  }
  std::vector<uint8_t> pkt;
  pkt.reserve(params.size() + 6);
  pkt.push_back(0xFF);
  pkt.push_back(0xFF);
  pkt.push_back(id);
  pkt.push_back(static_cast<uint8_t>(params.size() + 2));
  pkt.push_back(static_cast<uint8_t>(instruction));
  pkt.insert(pkt.end(), params.begin(), params.end());
  pkt.push_back(checksum(pkt.data() + 2, pkt.data() + pkt.size()));
  return pkt;
}

// ---------------------------------------------------------------------------
// PacketParser

void PacketParser::reset()
{
  state_ = State::kHeader1;
  body_.clear();
}

std::optional<PacketParser::Packet> PacketParser::push(uint8_t byte)
{
  switch (state_) {
    case State::kHeader1:
      if (byte == 0xFF) {
        state_ = State::kHeader2;
      }
      return std::nullopt;
    case State::kHeader2:
      state_ = (byte == 0xFF) ? State::kId : State::kHeader1;
      return std::nullopt;
    case State::kId:
      if (byte == 0xFF) {
        // Tolerate extra header bytes ("FF FF FF id ...").
        return std::nullopt;
      }
      id_ = byte;
      state_ = State::kLength;
      return std::nullopt;
    case State::kLength:
      if (byte < 2) {
        // Shortest legal packet carries code + checksum.
        reset();
        return std::nullopt;
      }
      length_ = byte;
      body_.clear();
      body_.reserve(length_);
      state_ = State::kBody;
      return std::nullopt;
    case State::kBody:
      body_.push_back(byte);
      if (body_.size() < length_) {
        return std::nullopt;
      }
      {
        // Checksum covers ID, LEN, code and params.
        unsigned sum = id_ + length_;
        for (std::size_t i = 0; i + 1 < body_.size(); ++i) {
          sum += body_[i];
        }
        const uint8_t expected = static_cast<uint8_t>(~sum & 0xFF);
        const uint8_t received = body_.back();
        state_ = State::kHeader1;
        if (expected != received) {
          ++checksum_failures_;
          body_.clear();
          return std::nullopt;
        }
        Packet pkt;
        pkt.id = id_;
        pkt.code = body_.front();
        pkt.params.assign(body_.begin() + 1, body_.end() - 1);
        body_.clear();
        return pkt;
      }
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// Sign-magnitude

uint16_t encode_sign_magnitude(int value, int sign_bit)
{
  const int max_magnitude = (1 << sign_bit) - 1;
  const int magnitude = std::abs(value);
  if (magnitude > max_magnitude) {
    throw std::out_of_range("feetech: magnitude does not fit below sign bit");
  }
  return static_cast<uint16_t>((value < 0 ? (1 << sign_bit) : 0) | magnitude);
}

int decode_sign_magnitude(uint16_t raw, int sign_bit)
{
  const int magnitude = raw & ((1 << sign_bit) - 1);
  return (raw & (1 << sign_bit)) ? -magnitude : magnitude;
}

// ---------------------------------------------------------------------------
// Bus

Bus::Bus(Transport & transport, std::chrono::microseconds timeout)
: transport_(transport), timeout_(timeout)
{
}

void Bus::send(uint8_t id, Instruction instruction, const std::vector<uint8_t> & params)
{
  const auto pkt = make_instruction(id, instruction, params);
  // Drop stale bytes (late replies from a previous timed-out transaction) so
  // they cannot be mistaken for the answer to this request.
  transport_.flush_input();
  parser_.reset();
  transport_.write(pkt.data(), pkt.size());
}

std::optional<StatusPacket> Bus::receive(std::chrono::steady_clock::time_point deadline)
{
  uint8_t byte = 0;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return std::nullopt;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    // Byte-at-a-time keeps packet boundaries exact: during a sync read several
    // replies arrive back to back, and a chunked read could swallow the start
    // of the next servo's packet. At 1 Mbaud a full 6-servo reply is ~70 bytes,
    // so the extra syscalls are negligible next to the bus transfer time.
    if (transport_.read(&byte, 1, remaining) == 0) {
      continue;
    }
    if (auto pkt = parser_.push(byte)) {
      last_error_[pkt->id] = pkt->code;
      return StatusPacket{pkt->id, pkt->code, std::move(pkt->params)};
    }
  }
}

StatusPacket Bus::transact(uint8_t id, Instruction instruction, const std::vector<uint8_t> & params)
{
  send(id, instruction, params);
  const auto deadline = std::chrono::steady_clock::now() + timeout_;
  while (true) {
    auto status = receive(deadline);
    if (!status) {
      throw CommError("feetech: no reply from servo " + std::to_string(id));
    }
    if (status->id == id) {
      return *status;
    }
    // A packet from another ID (e.g. a straggler); keep waiting.
  }
}

bool Bus::ping(uint8_t id)
{
  try {
    transact(id, Instruction::kPing, {});
    return true;
  } catch (const CommError &) {
    return false;
  }
}

std::vector<uint8_t> Bus::read(uint8_t id, uint8_t address, uint8_t length)
{
  auto status = transact(id, Instruction::kRead, {address, length});
  if (status.params.size() != length) {
    throw CommError(
            "feetech: servo " + std::to_string(id) + " returned " +
            std::to_string(status.params.size()) + " bytes, expected " + std::to_string(length));
  }
  return status.params;
}

uint8_t Bus::read_u8(uint8_t id, uint8_t address)
{
  return read(id, address, 1)[0];
}

uint16_t Bus::read_u16(uint8_t id, uint8_t address)
{
  const auto d = read(id, address, 2);
  return from_u16_le(d.data());
}

void Bus::write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params;
  params.reserve(data.size() + 1);
  params.push_back(address);
  params.insert(params.end(), data.begin(), data.end());
  if (id == kBroadcastId) {
    send(id, Instruction::kWrite, params);
    return;
  }
  transact(id, Instruction::kWrite, params);
}

void Bus::write_u8(uint8_t id, uint8_t address, uint8_t value)
{
  write(id, address, {value});
}

void Bus::write_u16(uint8_t id, uint8_t address, uint16_t value)
{
  write(id, address, u16_le(value));
}

void Bus::sync_write(
  uint8_t address, uint8_t length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & id_data)
{
  if (id_data.empty()) {
    return;
  }
  std::vector<uint8_t> params;
  params.reserve(2 + id_data.size() * (1 + length));
  params.push_back(address);
  params.push_back(length);
  for (const auto & [id, data] : id_data) {
    if (data.size() != length) {
      throw std::invalid_argument("feetech: sync_write data length mismatch");
    }
    params.push_back(id);
    params.insert(params.end(), data.begin(), data.end());
  }
  send(kBroadcastId, Instruction::kSyncWrite, params);
}

std::vector<std::optional<std::vector<uint8_t>>> Bus::sync_read(
  uint8_t address, uint8_t length, const std::vector<uint8_t> & ids)
{
  std::vector<std::optional<std::vector<uint8_t>>> out(ids.size());
  if (ids.empty()) {
    return out;
  }
  std::vector<uint8_t> params{address, length};
  params.insert(params.end(), ids.begin(), ids.end());
  send(kBroadcastId, Instruction::kSyncRead, params);

  // One shared deadline, scaled with the number of servos expected to reply.
  const auto deadline = std::chrono::steady_clock::now() + timeout_ * static_cast<int>(ids.size());
  std::size_t remaining = ids.size();
  while (remaining > 0) {
    auto status = receive(deadline);
    if (!status) {
      break;
    }
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == status->id && !out[i] && status->params.size() == length) {
        out[i] = std::move(status->params);
        --remaining;
        break;
      }
    }
  }
  return out;
}

}  // namespace so101_hardware::feetech
