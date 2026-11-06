// so101_bus_tool: check the servo bus before starting ROS 2.
//
//   so101_bus_tool --port /dev/ttyACM0 scan
//       Ping IDs 1..12 and print model, position, voltage, temperature.
//   so101_bus_tool --port /dev/ttyACM0 --calibration <lerobot.json> joints
//       Print each calibrated joint in raw steps, radians and degrees, and
//       whether the servo's stored homing offset matches the file.
//
// Read-only: it never enables torque or moves the arm.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech/protocol.hpp"
#include "so101_hardware/feetech/serial_port.hpp"
#include "so101_hardware/feetech/sts3215.hpp"
#include "so101_hardware/joint_mapping.hpp"

using namespace so101_hardware;
namespace reg = feetech::sts3215;

namespace
{
[[noreturn]] void usage()
{
  std::fprintf(
    stderr,
    "usage: so101_bus_tool --port DEV [--baud N] scan\n"
    "       so101_bus_tool --port DEV [--baud N] --calibration FILE joints\n");
  std::exit(2);
}

int read_position(feetech::Bus & bus, uint8_t id)
{
  return feetech::decode_sign_magnitude(
    bus.read_u16(id, reg::PRESENT_POSITION), reg::kPositionSignBit);
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string port, calibration_file, command;
  int baud = 1000000;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (a == "--baud" && i + 1 < argc) {
      baud = std::atoi(argv[++i]);
    } else if (a == "--calibration" && i + 1 < argc) {
      calibration_file = argv[++i];
    } else if (command.empty() && (a == "scan" || a == "joints")) {
      command = a;
    } else {
      usage();
    }
  }
  if (port.empty() || command.empty() || (command == "joints" && calibration_file.empty())) {
    usage();
  }

  try {
    feetech::SerialPort serial(port, baud);
    feetech::Bus bus(serial, std::chrono::milliseconds(20));

    if (command == "scan") {
      std::printf("%-4s %-7s %-9s %-8s %-6s\n", "id", "model", "position", "voltage", "temp");
      int found = 0;
      for (uint8_t id = 1; id <= 12; ++id) {
        if (!bus.ping(id)) {
          continue;
        }
        ++found;
        const unsigned model = bus.read_u16(id, reg::MODEL_NUMBER);
        const int pos = read_position(bus, id);
        const double volts = bus.read_u8(id, reg::PRESENT_VOLTAGE) / 10.0;
        const int temp = bus.read_u8(id, reg::PRESENT_TEMPERATURE);
        std::printf(
          "%-4d %-7u %-9d %-8.1f %-6d%s\n", id, model, pos, volts, temp,
          model == reg::kModelNumber ? "" : "  (not an STS3215)");
      }
      std::printf("%d servo(s) found on %s\n", found, port.c_str());
      return found > 0 ? 0 : 1;
    }

    const Calibration cal = load_calibration(calibration_file);
    std::printf(
      "%-14s %-3s %-6s %-9s %-8s %-s\n", "motor", "id", "raw", "rad", "deg", "homing offset");
    int problems = 0;
    for (const auto & [name, c] : cal) {
      const auto id = static_cast<uint8_t>(c.id);
      try {
        const int raw = read_position(bus, id);
        const int offset = feetech::decode_sign_magnitude(
          bus.read_u16(id, reg::HOMING_OFFSET), reg::kHomingOffsetSignBit);
        // Joint conversion for display uses the revolute mapping; the gripper
        // is shown on the same scale for comparison with LeRobot degrees.
        const JointMapping m(MappingKind::kRevolute, c, -10, 10);
        const double rad = m.raw_to_position(raw);
        const bool ok = offset == c.homing_offset;
        problems += ok ? 0 : 1;
        std::printf(
          "%-14s %-3d %-6d %+-9.4f %+-8.2f %d %s\n", name.c_str(), c.id, raw, rad,
          rad * 180.0 / M_PI, offset, ok ? "ok" : "MISMATCH (wrong arm or stale calibration?)");
      } catch (const feetech::CommError & e) {
        ++problems;
        std::printf("%-14s %-3d no reply\n", name.c_str(), c.id);
      }
    }
    return problems == 0 ? 0 : 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
