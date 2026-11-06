#include "so101_hardware/feetech/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif

#if defined(__linux__)
#include <linux/serial.h>
#endif

namespace so101_hardware::feetech
{

namespace
{

std::string errno_message(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

#if defined(__linux__)
speed_t standard_speed(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return 0;
  }
}
#else
speed_t standard_speed(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return 0;
  }
}
#endif

}  // namespace

SerialPort::SerialPort(const std::string & device, int baudrate)
: device_(device)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw TransportError(errno_message("cannot open " + device));
  }
  try {
    configure(baudrate);
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}

SerialPort::~SerialPort()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void SerialPort::configure(int baudrate)
{
  // Refuse to share the bus with another process (e.g. a LeRobot script):
  // interleaved packets from two hosts corrupt both control loops.
  if (::ioctl(fd_, TIOCEXCL) != 0 && errno != ENOTTY) {
    throw TransportError(errno_message("cannot get exclusive access to " + device_));
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    throw TransportError(errno_message("tcgetattr " + device_));
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  const speed_t speed = standard_speed(baudrate);
#if defined(__APPLE__)
  // macOS: set a placeholder rate with termios, then the real one via ioctl.
  ::cfsetispeed(&tio, speed ? speed : B115200);
  ::cfsetospeed(&tio, speed ? speed : B115200);
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    throw TransportError(errno_message("tcsetattr " + device_));
  }
  if (!speed) {
    speed_t custom = static_cast<speed_t>(baudrate);
    // Pseudo-terminals (used by the simulated bus) reject IOSSIOSPEED; the
    // rate is meaningless there, so only fail on real devices.
    if (::ioctl(fd_, IOSSIOSPEED, &custom) != 0 && errno != ENOTTY && errno != EINVAL) {
      throw TransportError(errno_message("IOSSIOSPEED " + device_));
    }
  }
#else
  if (!speed) {
    throw TransportError("unsupported baud rate " + std::to_string(baudrate));
  }
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    throw TransportError(errno_message("tcsetattr " + device_));
  }
#endif

#if defined(__linux__)
  // USB-serial bridges batch input for up to 16 ms by default; ask for
  // low-latency mode so each servo reply is delivered immediately. Not all
  // drivers support it, so failure is not fatal.
  serial_struct ss{};
  if (::ioctl(fd_, TIOCGSERIAL, &ss) == 0) {
    ss.flags |= ASYNC_LOW_LATENCY;
    ::ioctl(fd_, TIOCSSERIAL, &ss);
  }
#endif

  ::tcflush(fd_, TCIOFLUSH);
}

void SerialPort::write(const uint8_t * data, std::size_t size)
{
  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd_, data + written, size - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      pollfd pfd{fd_, POLLOUT, 0};
      ::poll(&pfd, 1, 10);
      continue;
    }
    throw TransportError(errno_message("write " + device_));
  }
}

std::size_t SerialPort::read(uint8_t * data, std::size_t size, std::chrono::microseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t got = 0;
  while (got < size) {
    const ssize_t n = ::read(fd_, data + got, size - got);
    if (n > 0) {
      got += static_cast<std::size_t>(n);
      continue;
    }
    if (n == 0 || errno == EAGAIN || errno == EINTR) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const auto left_us =
        std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
      pollfd pfd{fd_, POLLIN, 0};
      // poll() has millisecond resolution; round up so we never spin.
      const int left_ms = static_cast<int>((left_us + 999) / 1000);
      const int r = ::poll(&pfd, 1, left_ms);
      if (r < 0 && errno != EINTR) {
        throw TransportError(errno_message("poll " + device_));
      }
      if (r > 0 && (pfd.revents & (POLLERR | POLLNVAL))) {
        throw TransportError("serial device error on " + device_ + " (unplugged?)");
      }
      if (r > 0 && (pfd.revents & POLLHUP) && !(pfd.revents & POLLIN)) {
        throw TransportError("serial device hung up: " + device_);
      }
      continue;
    }
    throw TransportError(errno_message("read " + device_));
  }
  return got;
}

void SerialPort::flush_input()
{
  ::tcflush(fd_, TCIFLUSH);
}

}  // namespace so101_hardware::feetech
