// ros2_control SystemInterface for the SO-101 arm (six STS3215 servos on one
// half-duplex serial bus).
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "so101_hardware/feetech/protocol.hpp"
#include "so101_hardware/feetech/transport.hpp"
#include "so101_hardware/joint_mapping.hpp"

namespace so101_hardware
{

struct HardwareParams
{
  std::string port{"/dev/ttyACM0"};
  int baudrate{1000000};
  std::string calibration_file;
  int timeout_us{20000};
  bool verify_homing_offset{true};
  bool torque_off_on_deactivate{true};
  int max_consecutive_read_failures{5};
  // Position loop gains written to each servo (same defaults as LeRobot's
  // SO follower). Negative disables writing that gain.
  int p_coefficient{16};
  int i_coefficient{0};
  int d_coefficient{32};
};

class SO101SystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using TransportFactory =
    std::function<std::unique_ptr<feetech::Transport>(const HardwareParams &)>;

  SO101SystemHardware();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Replace how the serial transport is created (tests inject a simulator).
  void set_transport_factory(TransportFactory factory) { transport_factory_ = std::move(factory); }

  const HardwareParams & params() const { return params_; }

private:
  struct Joint
  {
    std::string name;
    uint8_t id{0};
    JointMapping mapping;
    double position{0.0};
    double velocity{0.0};
    double command{0.0};
    int last_goal_raw{-1};
  };

  bool read_all_positions_blocking();
  void set_torque(bool enable);
  void apply_gains(Joint & joint);
  void close_bus();

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  HardwareParams params_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> ids_;
  TransportFactory transport_factory_;
  std::unique_ptr<feetech::Transport> transport_;
  std::unique_ptr<feetech::Bus> bus_;
  int consecutive_read_failures_{0};
  bool torque_enabled_{false};
};

}  // namespace so101_hardware
