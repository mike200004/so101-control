// POSIX serial port transport (Linux and macOS).
#pragma once

#include <string>

#include "so101_hardware/feetech/transport.hpp"

namespace so101_hardware::feetech
{

class SerialPort final : public Transport
{
public:
  /// Opens `device` in raw 8N1 mode at `baudrate`. Throws TransportError.
  /// On Linux the rate must have a termios constant (1 Mbaud, the STS3215
  /// default, is B1000000); other rates throw. On macOS any rate is set
  /// through IOSSIOSPEED.
  SerialPort(const std::string & device, int baudrate);
  ~SerialPort() override;

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void write(const uint8_t * data, std::size_t size) override;
  std::size_t read(uint8_t * data, std::size_t size, std::chrono::microseconds timeout) override;
  void flush_input() override;

  const std::string & device() const { return device_; }

private:
  void configure(int baudrate);

  std::string device_;
  int fd_{-1};
};

}  // namespace so101_hardware::feetech
