// fake_servo_bus: expose a simulated SO-101 servo chain on a pseudo-terminal.
//
//   fake_servo_bus --calibration <lerobot.json> [--link /tmp/so101_fake_bus]
//
// The hardware plugin (or any Feetech client) can then open the link path as
// if it were the arm's USB serial port. Useful for CI and for trying the ROS 2
// stack without an arm on the desk.
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/sim/servo_bus_sim.hpp"

namespace
{
std::atomic<bool> g_running{true};
void on_signal(int) {g_running = false;}

[[noreturn]] void usage()
{
  std::fprintf(
    stderr,
    "usage: fake_servo_bus --calibration FILE [--link PATH] [--speed STEPS_PER_S]\n");
  std::exit(2);
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string calibration_file;
  std::string link = "/tmp/so101_fake_bus";
  double speed = 4000.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--calibration" && i + 1 < argc) {
      calibration_file = argv[++i];
    } else if (a == "--link" && i + 1 < argc) {
      link = argv[++i];
    } else if (a == "--speed" && i + 1 < argc) {
      speed = std::atof(argv[++i]);
    } else {
      usage();
    }
  }
  if (calibration_file.empty()) {
    usage();
  }

  so101_hardware::sim::ServoBusSim sim(so101_hardware::load_calibration(calibration_file));

  const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0) {
    std::perror("posix_openpt");
    return 1;
  }
  const char * slave_name = ::ptsname(master);
  // Hold the slave open ourselves so the master never sees a hangup between
  // client connections, and put it in raw mode so nothing is echoed.
  const int slave = ::open(slave_name, O_RDWR | O_NOCTTY);
  termios tio{};
  ::tcgetattr(slave, &tio);
  ::cfmakeraw(&tio);
  ::tcsetattr(slave, TCSANOW, &tio);

  ::unlink(link.c_str());
  if (::symlink(slave_name, link.c_str()) != 0) {
    std::perror("symlink");
    return 1;
  }
  std::printf("fake_servo_bus: %s -> %s\n", link.c_str(), slave_name);
  std::fflush(stdout);

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  auto last = std::chrono::steady_clock::now();
  uint8_t buf[512];
  while (g_running) {
    pollfd pfd{master, POLLIN, 0};
    const int r = ::poll(&pfd, 1, 2);
    if (r > 0 && (pfd.revents & POLLIN)) {
      const ssize_t n = ::read(master, buf, sizeof(buf));
      if (n > 0) {
        const auto reply = sim.handle(buf, static_cast<std::size_t>(n));
        if (!reply.empty() && ::write(master, reply.data(), reply.size()) < 0) {
          std::perror("write");
        }
      }
    }
    const auto now = std::chrono::steady_clock::now();
    sim.step(std::chrono::duration<double>(now - last).count(), speed);
    last = now;
  }

  ::unlink(link.c_str());
  ::close(slave);
  ::close(master);
  return 0;
}
