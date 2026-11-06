// Byte transport abstraction for the Feetech half-duplex servo bus.
//
// The protocol layer only needs "write these bytes" and "read up to N bytes
// before a deadline". Keeping that behind an interface lets the same protocol
// code run against a real serial port, a pseudo-terminal, or an in-process
// simulated servo bus in unit tests.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace so101_hardware::feetech
{

/// Raised for unrecoverable transport failures (port closed, OS error).
class TransportError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class Transport
{
public:
  virtual ~Transport() = default;

  /// Write all bytes or throw TransportError.
  virtual void write(const uint8_t * data, std::size_t size) = 0;

  /// Read up to `size` bytes, blocking at most `timeout` in total.
  /// Returns the number of bytes actually read (0 on timeout).
  virtual std::size_t read(uint8_t * data, std::size_t size, std::chrono::microseconds timeout) = 0;

  /// Discard any unread input (stale replies, line noise).
  virtual void flush_input() = 0;
};

}  // namespace so101_hardware::feetech
