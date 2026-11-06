#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/joint_mapping.hpp"

using namespace so101_hardware;

namespace
{
constexpr double kPi = 3.141592653589793;

MotorCalibration cal(int min, int max, int drive_mode = 0)
{
  MotorCalibration c;
  c.id = 1;
  c.range_min = min;
  c.range_max = max;
  c.drive_mode = drive_mode;
  return c;
}

/// LeRobot 0.6 `MotorNormMode.DEGREES`, from motors_bus.py::_normalize.
double lerobot_degrees(int raw, int min, int max)
{
  const double mid = (min + max) / 2.0;
  return (raw - mid) * 360.0 / 4095.0;
}
}  // namespace

TEST(Revolute, MiddleOfRangeIsZero)
{
  JointMapping m(MappingKind::kRevolute, cal(697, 3286), -2.0, 2.0);
  EXPECT_NEAR(m.raw_to_position(1991), -0.5 * 2 * kPi / 4096, 1e-12);  // mid is 1991.5
  EXPECT_NEAR(m.raw_to_position(1992), 0.5 * 2 * kPi / 4096, 1e-12);
}

TEST(Revolute, QuarterTurnIs1024Steps)
{
  JointMapping m(MappingKind::kRevolute, cal(0, 4096 - 2), -4.0, 4.0);  // mid 2047
  EXPECT_NEAR(m.raw_to_position(2047 + 1024), kPi / 2, 1e-12);
  EXPECT_NEAR(m.raw_to_position(2047 - 1024), -kPi / 2, 1e-12);
}

TEST(Revolute, DriveModeInvertsDirection)
{
  JointMapping fwd(MappingKind::kRevolute, cal(1000, 3000, 0), -4, 4);
  JointMapping rev(MappingKind::kRevolute, cal(1000, 3000, 1), -4, 4);
  EXPECT_NEAR(fwd.raw_to_position(2500), -rev.raw_to_position(2500), 1e-12);
  EXPECT_EQ(rev.position_to_raw(rev.raw_to_position(2500)), 2500);
}

TEST(Revolute, RoundTripsEveryStepInRange)
{
  for (int dm : {0, 1}) {
    JointMapping m(MappingKind::kRevolute, cal(835, 3248, dm), -4, 4);
    for (int raw = 835; raw <= 3248; ++raw) {
      ASSERT_EQ(m.position_to_raw(m.raw_to_position(raw)), raw) << "drive_mode=" << dm;
    }
  }
}

TEST(Revolute, CommandsAreClampedToCalibratedRange)
{
  JointMapping m(MappingKind::kRevolute, cal(907, 3146), -4, 4);
  EXPECT_EQ(m.position_to_raw(100.0), 3146);
  EXPECT_EQ(m.position_to_raw(-100.0), 907);
}

TEST(Revolute, RejectsNonFiniteCommands)
{
  JointMapping m(MappingKind::kRevolute, cal(907, 3146), -4, 4);
  EXPECT_THROW(m.position_to_raw(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(Revolute, AgreesWithLeRobotDegreesMode)
{
  // LeRobot divides by 4095 instead of 4096 steps/rev; the difference is
  // 0.025 %, i.e. < 0.05 deg anywhere in the SO-101 workspace.
  const int min = 805, max = 3150;
  JointMapping m(MappingKind::kRevolute, cal(min, max), -4, 4);
  for (int raw = min; raw <= max; raw += 7) {
    const double ours_deg = m.raw_to_position(raw) * 180.0 / kPi;
    EXPECT_NEAR(ours_deg, lerobot_degrees(raw, min, max), 0.05) << raw;
  }
}

TEST(Linear, MapsCalibratedRangeOntoUrdfLimits)
{
  const double lower = -0.174533, upper = 1.74533;
  JointMapping m(MappingKind::kLinear, cal(2020, 3466), lower, upper);
  EXPECT_NEAR(m.raw_to_position(2020), lower, 1e-9);
  EXPECT_NEAR(m.raw_to_position(3466), upper, 1e-9);
  EXPECT_NEAR(m.raw_to_position(2743), (lower + upper) / 2, 1e-9);
  EXPECT_EQ(m.position_to_raw(lower), 2020);
  EXPECT_EQ(m.position_to_raw(upper), 3466);
  EXPECT_EQ(m.position_to_raw(upper + 1.0), 3466);
}

TEST(Linear, DriveModeSwapsEnds)
{
  JointMapping m(MappingKind::kLinear, cal(2020, 3466, 1), 0.0, 1.0);
  EXPECT_NEAR(m.raw_to_position(2020), 1.0, 1e-9);
  EXPECT_NEAR(m.raw_to_position(3466), 0.0, 1e-9);
  EXPECT_EQ(m.position_to_raw(0.25), m.position_to_raw(m.raw_to_position(m.position_to_raw(0.25))));
}

TEST(Linear, RequiresFiniteLimits)
{
  EXPECT_THROW(
    JointMapping(
      MappingKind::kLinear, cal(0, 100), -std::numeric_limits<double>::infinity(), 1.0),
    std::invalid_argument);
}

TEST(MappingKind, Parses)
{
  EXPECT_EQ(parse_mapping_kind("revolute"), MappingKind::kRevolute);
  EXPECT_EQ(parse_mapping_kind("linear"), MappingKind::kLinear);
  EXPECT_THROW(parse_mapping_kind("prismatic"), std::invalid_argument);
}

// ---------------------------------------------------------------------------

TEST(Calibration, ParsesLeRobotFile)
{
  const auto c = parse_calibration(R"({
    "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 1788,
                     "range_min": 697, "range_max": 3286},
    "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 1252,
                "range_min": 2020, "range_max": 3466}
  })");
  ASSERT_EQ(c.size(), 2u);
  EXPECT_EQ(c.at("shoulder_pan").id, 1);
  EXPECT_EQ(c.at("shoulder_pan").homing_offset, 1788);
  EXPECT_EQ(c.at("gripper").range_max, 3466);
}

TEST(Calibration, ReportsMissingField)
{
  try {
    parse_calibration(R"({"elbow_flex": {"id": 3, "drive_mode": 0, "range_min": 1, "range_max": 2}})");
    FAIL() << "expected throw";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("homing_offset"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("elbow_flex"), std::string::npos);
  }
}

TEST(Calibration, RejectsEmptyRange)
{
  EXPECT_THROW(
    parse_calibration(
      R"({"m": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 5, "range_max": 5}})"),
    std::runtime_error);
}

TEST(Calibration, RejectsInvalidJson)
{
  EXPECT_THROW(parse_calibration("{not json"), std::runtime_error);
}
