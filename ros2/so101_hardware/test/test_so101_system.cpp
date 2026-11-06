// Lifecycle tests for the ros2_control plugin, run against the in-process
// servo bus simulator (same wire protocol as the real arm).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech/protocol.hpp"
#include "so101_hardware/feetech/sts3215.hpp"
#include "so101_hardware/sim/servo_bus_sim.hpp"
#include "so101_hardware/so101_system.hpp"

using namespace so101_hardware;
namespace reg = so101_hardware::feetech::sts3215;
using CallbackReturn = SO101SystemHardware::CallbackReturn;
using hardware_interface::return_type;

namespace
{

constexpr double kGripperLower = -0.174533;
constexpr double kGripperUpper = 1.74533;

// Forwards to a simulator owned by the test fixture, so the plugin can own a
// Transport while the test keeps inspecting the simulated servos.
class ForwardingTransport : public feetech::Transport
{
public:
  explicit ForwardingTransport(sim::ServoBusSim & sim) : inner_(sim) {}
  void write(const uint8_t * d, std::size_t n) override {inner_.write(d, n);}
  std::size_t read(uint8_t * d, std::size_t n, std::chrono::microseconds t) override
  {
    return inner_.read(d, n, t);
  }
  void flush_input() override {inner_.flush_input();}

private:
  sim::SimTransport inner_;
};

hardware_interface::InterfaceInfo iface(const std::string & name, double min = NAN, double max = NAN)
{
  hardware_interface::InterfaceInfo i;
  i.name = name;
  if (!std::isnan(min)) {
    i.min = std::to_string(min);
    i.max = std::to_string(max);
  }
  return i;
}

hardware_interface::HardwareInfo make_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "so101";
  info.type = "system";
  info.hardware_parameters["port"] = "/dev/null";
  info.hardware_parameters["calibration_file"] = EXAMPLE_CALIBRATION;
  info.hardware_parameters["timeout_us"] = "2000";
  info.hardware_parameters["max_consecutive_read_failures"] = "3";

  for (const char * name :
    {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"})
  {
    hardware_interface::ComponentInfo j;
    j.name = name;
    j.type = "joint";
    j.command_interfaces.push_back(iface(hardware_interface::HW_IF_POSITION, -3.0, 3.0));
    j.state_interfaces.push_back(iface(hardware_interface::HW_IF_POSITION));
    j.state_interfaces.push_back(iface(hardware_interface::HW_IF_VELOCITY));
    info.joints.push_back(j);
  }
  hardware_interface::ComponentInfo g;
  g.name = "gripper";
  g.type = "joint";
  g.parameters["mapping"] = "linear";
  g.command_interfaces.push_back(
    iface(hardware_interface::HW_IF_POSITION, kGripperLower, kGripperUpper));
  g.state_interfaces.push_back(iface(hardware_interface::HW_IF_POSITION));
  g.state_interfaces.push_back(iface(hardware_interface::HW_IF_VELOCITY));
  info.joints.push_back(g);
  return info;
}

const rclcpp_lifecycle::State kUnconfigured(
  lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
const rclcpp_lifecycle::State kInactive(
  lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
const rclcpp_lifecycle::State kActive(
  lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

}  // namespace

class SystemTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    calibration_ = load_calibration(EXAMPLE_CALIBRATION);
    sim_ = std::make_unique<sim::ServoBusSim>(calibration_);
    hw_ = std::make_unique<SO101SystemHardware>();
    hw_->set_transport_factory(
      [this](const HardwareParams &) {return std::make_unique<ForwardingTransport>(*sim_);});
  }

  void init_and_export(const hardware_interface::HardwareInfo & info = make_info())
  {
    ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
    states_ = hw_->export_state_interfaces();
    commands_ = hw_->export_command_interfaces();
  }

  double state(const std::string & joint, const std::string & iface_name = "position")
  {
    for (auto & s : states_) {
      if (s.get_name() == joint + "/" + iface_name) {
        return s.get_value();
      }
    }
    ADD_FAILURE() << "no state interface " << joint << "/" << iface_name;
    return NAN;
  }

  hardware_interface::CommandInterface & command(const std::string & joint)
  {
    for (auto & c : commands_) {
      if (c.get_prefix_name() == joint) {
        return c;
      }
    }
    throw std::runtime_error("no command interface for " + joint);
  }

  // set_value() returns void in early Jazzy releases and bool in later ones.
  void set_command(const std::string & joint, double value)
  {
    (void)command(joint).set_value(value);
  }

  int goal_raw(const std::string & motor)
  {
    return feetech::decode_sign_magnitude(
      sim_->reg_u16(calibration_.at(motor).id, reg::GOAL_POSITION), reg::kPositionSignBit);
  }

  void cycle(int n = 1)
  {
    const rclcpp::Time t(0, 0, RCL_STEADY_TIME);
    const rclcpp::Duration dt(0, 10'000'000);
    for (int i = 0; i < n; ++i) {
      ASSERT_EQ(hw_->write(t, dt), return_type::OK);
      sim_->step(0.01);
      ASSERT_EQ(hw_->read(t, dt), return_type::OK);
    }
  }

  Calibration calibration_;
  std::unique_ptr<sim::ServoBusSim> sim_;
  std::unique_ptr<SO101SystemHardware> hw_;
  std::vector<hardware_interface::StateInterface> states_;
  std::vector<hardware_interface::CommandInterface> commands_;
};

TEST_F(SystemTest, ExportsPositionAndVelocityForSixJoints)
{
  init_and_export();
  EXPECT_EQ(states_.size(), 12u);
  EXPECT_EQ(commands_.size(), 6u);
}

TEST_F(SystemTest, InitFailsWithoutCalibrationFile)
{
  auto info = make_info();
  info.hardware_parameters.erase("calibration_file");
  EXPECT_EQ(hw_->on_init(info), CallbackReturn::ERROR);
}

TEST_F(SystemTest, InitFailsForUnknownMotor)
{
  auto info = make_info();
  info.joints[0].name = "not_a_motor";
  EXPECT_EQ(hw_->on_init(info), CallbackReturn::ERROR);
}

TEST_F(SystemTest, ConfigureReadsPoseAtMiddleOfRange)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  // The simulator parks every servo at the middle of its calibrated range,
  // which is the URDF zero pose (within half a step of rounding).
  for (const char * j : {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"}) {
    EXPECT_NEAR(state(j), 0.0, 2e-3) << j;
  }
  EXPECT_NEAR(state("gripper"), (kGripperLower + kGripperUpper) / 2, 2e-3);
}

TEST_F(SystemTest, ConfigureFailsWhenAServoIsMissing)
{
  init_and_export();
  sim_->set_silent(4, true);
  EXPECT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::ERROR);
}

TEST_F(SystemTest, ConfigureRejectsHomingOffsetMismatch)
{
  init_and_export();
  // Servo 2 was recalibrated (or this is the other arm).
  sim_->set_reg_u16(2, reg::HOMING_OFFSET, feetech::encode_sign_magnitude(100, 11));
  EXPECT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::ERROR);
}

TEST_F(SystemTest, HomingCheckCanBeDisabled)
{
  auto info = make_info();
  info.hardware_parameters["verify_homing_offset"] = "false";
  init_and_export(info);
  sim_->set_reg_u16(2, reg::HOMING_OFFSET, feetech::encode_sign_magnitude(100, 11));
  EXPECT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
}

TEST_F(SystemTest, ActivateHoldsCurrentPoseWithoutJumping)
{
  init_and_export();
  sim_->set_position(3, 2600);  // elbow somewhere off-centre
  sim_->set_reg_u16(3, reg::GOAL_POSITION, 1000);  // stale goal from a previous session
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);

  EXPECT_EQ(goal_raw("elbow_flex"), 2600);
  EXPECT_EQ(sim_->reg_u8(3, reg::TORQUE_ENABLE), 1);
  EXPECT_DOUBLE_EQ(command("elbow_flex").get_value(), state("elbow_flex"));
  cycle(5);
  EXPECT_EQ(sim_->reg_u16(3, reg::PRESENT_POSITION), 2600);
}

TEST_F(SystemTest, GainsAreWrittenOnlyWhenTheyDiffer)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(sim_->reg_u8(1, reg::P_COEFFICIENT), 16);
  EXPECT_EQ(sim_->reg_u8(1, reg::D_COEFFICIENT), 32);
  EXPECT_EQ(sim_->reg_u8(1, reg::LOCK), 1);
  const int writes = sim_->eeprom_writes(1);
  EXPECT_GT(writes, 0);

  ASSERT_EQ(hw_->on_deactivate(kActive), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(sim_->eeprom_writes(1), writes) << "re-activation must not rewrite EEPROM";
}

TEST_F(SystemTest, CommandsDriveServosAndStateFollows)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);

  set_command("shoulder_lift", 0.5);
  set_command("gripper", kGripperUpper);
  cycle(100);  // 1 s of simulated time
  EXPECT_NEAR(state("shoulder_lift"), 0.5, 2e-3);
  EXPECT_NEAR(state("gripper"), kGripperUpper, 2e-3);
  EXPECT_EQ(goal_raw("gripper"), calibration_.at("gripper").range_max);
}

TEST_F(SystemTest, OutOfRangeCommandsAreClampedToCalibration)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);
  set_command("wrist_flex", 100.0);
  cycle();
  EXPECT_EQ(goal_raw("wrist_flex"), calibration_.at("wrist_flex").range_max);
}

TEST_F(SystemTest, UnchangedCommandsDoNotUseTheBus)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);
  set_command("shoulder_pan", 0.2);
  cycle();
  const int goal = goal_raw("shoulder_pan");
  sim_->set_reg_u16(1, reg::GOAL_POSITION, 1234);  // detect any rewrite
  cycle();
  EXPECT_EQ(goal_raw("shoulder_pan"), 1234);
  set_command("shoulder_pan", 0.3);
  cycle();
  EXPECT_NE(goal_raw("shoulder_pan"), 1234);
  EXPECT_NE(goal_raw("shoulder_pan"), goal);
}

TEST_F(SystemTest, ToleratesTransientDropoutsButFailsOnPersistentOnes)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);
  const rclcpp::Time t(0, 0, RCL_STEADY_TIME);
  const rclcpp::Duration dt(0, 10'000'000);

  sim_->set_silent(5, true);
  EXPECT_EQ(hw_->read(t, dt), return_type::OK);
  EXPECT_EQ(hw_->read(t, dt), return_type::OK);
  sim_->set_silent(5, false);
  EXPECT_EQ(hw_->read(t, dt), return_type::OK);  // recovered, counter resets

  sim_->set_silent(5, true);
  EXPECT_EQ(hw_->read(t, dt), return_type::OK);
  EXPECT_EQ(hw_->read(t, dt), return_type::OK);
  EXPECT_EQ(hw_->read(t, dt), return_type::ERROR);  // 3rd consecutive failure
}

TEST_F(SystemTest, DeactivateReleasesTorque)
{
  init_and_export();
  ASSERT_EQ(hw_->on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(kInactive), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_deactivate(kActive), CallbackReturn::SUCCESS);
  for (uint8_t id = 1; id <= 6; ++id) {
    EXPECT_EQ(sim_->reg_u8(id, reg::TORQUE_ENABLE), 0) << int(id);
  }
}
