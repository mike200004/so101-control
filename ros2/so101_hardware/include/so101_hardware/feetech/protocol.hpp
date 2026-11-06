// Feetech SCS/STS servo bus protocol ("protocol 0", used by the STS3215).
//
// Instruction packet:  FF FF | ID | LEN | INSTR | P1..Pn | CHK
// Status packet:       FF FF | ID | LEN | ERROR | P1..Pn | CHK
//   LEN = n + 2,  CHK = ~(ID + LEN + INSTR/ERROR + sum(P)) & 0xFF
// Multi-byte register values are little-endian on STS/SMS servos.
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "so101_hardware/feetech/transport.hpp"

namespace so101_hardware::feetech
{

constexpr uint8_t kBroadcastId = 0xFE;

enum class Instruction : uint8_t
{
  kPing = 0x01,
  kRead = 0x02,
  kWrite = 0x03,
  kRegWrite = 0x04,
  kAction = 0x05,
  kSyncRead = 0x82,
  kSyncWrite = 0x83,
};

/// Bits of the status packet ERROR byte.
namespace status_error
{
constexpr uint8_t kVoltage = 0x01;
constexpr uint8_t kAngle = 0x02;
constexpr uint8_t kOverheat = 0x04;
constexpr uint8_t kOverCurrent = 0x08;
constexpr uint8_t kOverload = 0x20;
}  // namespace status_error

/// Human-readable description of a status ERROR byte, e.g. "overheat|overload".
std::string describe_status_error(uint8_t error);

/// A bus transaction failed (timeout, bad checksum, wrong ID, short reply).
class CommError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct StatusPacket
{
  uint8_t id{0};
  uint8_t error{0};
  std::vector<uint8_t> params;
};

/// Checksum over ID..last parameter (everything except the FF FF header).
uint8_t checksum(const uint8_t * begin, const uint8_t * end);

/// Serialize an instruction packet.
std::vector<uint8_t> make_instruction(
  uint8_t id, Instruction instruction, const std::vector<uint8_t> & params);

/// Incremental status/instruction packet parser. Feed it bytes as they arrive;
/// it resynchronizes on the FF FF header and rejects bad checksums.
class PacketParser
{
public:
  struct Packet
  {
    uint8_t id;
    uint8_t code;  // instruction (host->servo) or error byte (servo->host)
    std::vector<uint8_t> params;
  };

  /// Push one byte. Returns a packet when one completes successfully.
  std::optional<Packet> push(uint8_t byte);
  void reset();
  /// Number of packets dropped because of a checksum mismatch.
  std::size_t checksum_failures() const { return checksum_failures_; }

private:
  enum class State { kHeader1, kHeader2, kId, kLength, kBody };
  State state_{State::kHeader1};
  uint8_t id_{0};
  uint8_t length_{0};
  std::vector<uint8_t> body_;
  std::size_t checksum_failures_{0};
};

/// Little-endian packing helpers.
inline std::vector<uint8_t> u16_le(uint16_t v)
{
  return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF)};
}
inline uint16_t from_u16_le(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

/// Sign-magnitude encoding used by several STS registers: bit `sign_bit` is
/// the sign, lower bits are the magnitude (it is NOT two's complement).
uint16_t encode_sign_magnitude(int value, int sign_bit);
int decode_sign_magnitude(uint16_t raw, int sign_bit);

/// Synchronous request/response access to a bus of servos.
class Bus
{
public:
  explicit Bus(Transport & transport, std::chrono::microseconds timeout = std::chrono::milliseconds(20));

  void set_timeout(std::chrono::microseconds timeout) { timeout_ = timeout; }

  /// True if `id` answers a PING.
  bool ping(uint8_t id);

  /// Read `length` bytes starting at `address`. Throws CommError.
  std::vector<uint8_t> read(uint8_t id, uint8_t address, uint8_t length);
  uint8_t read_u8(uint8_t id, uint8_t address);
  uint16_t read_u16(uint8_t id, uint8_t address);

  /// Write bytes starting at `address`, waiting for the status reply.
  void write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data);
  void write_u8(uint8_t id, uint8_t address, uint8_t value);
  void write_u16(uint8_t id, uint8_t address, uint16_t value);

  /// Write the same register range on many servos in one packet (no replies).
  void sync_write(
    uint8_t address, uint8_t length,
    const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & id_data);

  /// Read the same register range from many servos in one request. Each
  /// servo replies with its own status packet. Entries are std::nullopt for
  /// servos that did not answer in time or answered with a corrupt packet,
  /// so the caller can decide how to treat a partial read.
  std::vector<std::optional<std::vector<uint8_t>>> sync_read(
    uint8_t address, uint8_t length, const std::vector<uint8_t> & ids);

  /// ERROR byte of the most recent status packet received from each ID
  /// (indexed by ID, 0 if never seen).
  uint8_t last_status_error(uint8_t id) const { return last_error_[id]; }

private:
  void send(uint8_t id, Instruction instruction, const std::vector<uint8_t> & params);
  /// Wait for one status packet; returns nullopt on timeout.
  std::optional<StatusPacket> receive(std::chrono::steady_clock::time_point deadline);
  StatusPacket transact(uint8_t id, Instruction instruction, const std::vector<uint8_t> & params);

  Transport & transport_;
  std::chrono::microseconds timeout_;
  PacketParser parser_;
  uint8_t last_error_[256]{};
};

}  // namespace so101_hardware::feetech
