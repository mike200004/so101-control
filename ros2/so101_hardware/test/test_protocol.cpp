#include <gtest/gtest.h>

#include <vector>

#include "so101_hardware/feetech/protocol.hpp"
#include "so101_hardware/feetech/sts3215.hpp"
#include "so101_hardware/sim/servo_bus_sim.hpp"

using namespace so101_hardware;
using namespace so101_hardware::feetech;
namespace reg = so101_hardware::feetech::sts3215;

TEST(Packet, PingMatchesDatasheetBytes)
{
  // PING servo 1: FF FF 01 02 01 FB
  EXPECT_EQ(make_instruction(1, Instruction::kPing, {}),
    (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(Packet, ReadPresentPositionBytes)
{
  // READ servo 1, address 56, 2 bytes: FF FF 01 04 02 38 02 BE
  EXPECT_EQ(make_instruction(1, Instruction::kRead, {reg::PRESENT_POSITION, 2}),
    (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
}

TEST(Parser, ResynchronizesAfterGarbage)
{
  PacketParser parser;
  std::vector<uint8_t> stream{0x00, 0xFF, 0x13, 0xFF};  // noise, false header
  auto pkt = make_instruction(3, static_cast<Instruction>(0), {0x10, 0x20});
  stream.insert(stream.end(), pkt.begin(), pkt.end());

  std::optional<PacketParser::Packet> got;
  for (auto b : stream) {
    if (auto p = parser.push(b)) {
      got = p;
    }
  }
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->id, 3);
  EXPECT_EQ(got->code, 0);
  EXPECT_EQ(got->params, (std::vector<uint8_t>{0x10, 0x20}));
}

TEST(Parser, RejectsBadChecksum)
{
  PacketParser parser;
  auto pkt = make_instruction(1, Instruction::kPing, {});
  pkt.back() ^= 0x01;
  for (auto b : pkt) {
    EXPECT_FALSE(parser.push(b).has_value());
  }
  EXPECT_EQ(parser.checksum_failures(), 1u);
  // The parser recovers for the next, valid packet.
  std::optional<PacketParser::Packet> got;
  for (auto b : make_instruction(1, Instruction::kPing, {})) {
    if (auto p = parser.push(b)) {
      got = p;
    }
  }
  EXPECT_TRUE(got.has_value());
}

TEST(SignMagnitude, RoundTripsAndIsNotTwosComplement)
{
  EXPECT_EQ(encode_sign_magnitude(5, 15), 0x0005);
  EXPECT_EQ(encode_sign_magnitude(-5, 15), 0x8005);
  EXPECT_EQ(decode_sign_magnitude(0x8005, 15), -5);
  EXPECT_EQ(encode_sign_magnitude(-1890, 11), 0x0800 | 1890);
  EXPECT_EQ(decode_sign_magnitude(0x0800 | 1890, 11), -1890);
  for (int v = -2047; v <= 2047; v += 89) {
    EXPECT_EQ(decode_sign_magnitude(encode_sign_magnitude(v, 11), 11), v);
  }
  EXPECT_THROW(encode_sign_magnitude(2048, 11), std::out_of_range);
}

TEST(StatusError, DescribesBits)
{
  EXPECT_EQ(describe_status_error(0), "ok");
  EXPECT_EQ(
    describe_status_error(status_error::kOverheat | status_error::kOverload),
    "overheat|overload");
}

class BusTest : public ::testing::Test
{
protected:
  sim::ServoBusSim sim{{1, 2, 3, 4, 5, 6}};
  sim::SimTransport transport{sim};
  Bus bus{transport, std::chrono::milliseconds(2)};
};

TEST_F(BusTest, PingFindsOnlyPresentServos)
{
  EXPECT_TRUE(bus.ping(1));
  EXPECT_TRUE(bus.ping(6));
  EXPECT_FALSE(bus.ping(7));
}

TEST_F(BusTest, ReadsModelNumber)
{
  EXPECT_EQ(bus.read_u16(3, reg::MODEL_NUMBER), reg::kModelNumber);
}

TEST_F(BusTest, WriteThenRead)
{
  bus.write_u16(2, reg::GOAL_POSITION, 3000);
  EXPECT_EQ(bus.read_u16(2, reg::GOAL_POSITION), 3000);
}

TEST_F(BusTest, EepromWritesRespectLock)
{
  bus.write_u8(1, reg::P_COEFFICIENT, 16);  // locked: ignored, like hardware
  EXPECT_EQ(bus.read_u8(1, reg::P_COEFFICIENT), 32);
  bus.write_u8(1, reg::LOCK, 0);
  bus.write_u8(1, reg::P_COEFFICIENT, 16);
  EXPECT_EQ(bus.read_u8(1, reg::P_COEFFICIENT), 16);
}

TEST_F(BusTest, SyncWriteReachesEveryServoWithoutReplies)
{
  bus.sync_write(
    reg::GOAL_POSITION, 2,
    {{1, u16_le(1000)}, {4, u16_le(2000)}, {6, u16_le(3000)}});
  EXPECT_EQ(sim.reg_u16(1, reg::GOAL_POSITION), 1000);
  EXPECT_EQ(sim.reg_u16(4, reg::GOAL_POSITION), 2000);
  EXPECT_EQ(sim.reg_u16(6, reg::GOAL_POSITION), 3000);
  EXPECT_EQ(sim.reg_u16(2, reg::GOAL_POSITION), 0);
}

TEST_F(BusTest, SyncReadReturnsEachServo)
{
  sim.set_position(1, 100);
  sim.set_position(5, 4000);
  auto r = bus.sync_read(reg::PRESENT_POSITION, 2, {1, 5});
  ASSERT_TRUE(r[0] && r[1]);
  EXPECT_EQ(from_u16_le(r[0]->data()), 100);
  EXPECT_EQ(from_u16_le(r[1]->data()), 4000);
}

TEST_F(BusTest, SyncReadReportsMissingServoAsNullopt)
{
  sim.set_silent(3, true);
  auto r = bus.sync_read(reg::PRESENT_POSITION, 2, {2, 3, 4});
  EXPECT_TRUE(r[0].has_value());
  EXPECT_FALSE(r[1].has_value());
  EXPECT_TRUE(r[2].has_value());
}

TEST_F(BusTest, ReadFromSilentServoThrows)
{
  sim.set_silent(2, true);
  EXPECT_THROW(bus.read_u16(2, reg::PRESENT_POSITION), CommError);
}

TEST_F(BusTest, RecordsStatusErrorByte)
{
  sim.set_status_error(4, status_error::kOverload);
  bus.ping(4);
  EXPECT_EQ(bus.last_status_error(4), status_error::kOverload);
  EXPECT_EQ(bus.last_status_error(1), 0);
}

TEST(SimMotion, TorquedServoMovesTowardGoal)
{
  sim::ServoBusSim sim{{1}};
  sim.set_reg_u16(1, reg::GOAL_POSITION, 2148);
  sim.step(0.01);  // torque off: no motion
  EXPECT_EQ(sim.reg_u16(1, reg::PRESENT_POSITION), 2048);
  sim.set_reg_u8(1, reg::TORQUE_ENABLE, 1);
  sim.step(0.01, 4000.0);  // at most 40 steps
  EXPECT_EQ(sim.reg_u16(1, reg::PRESENT_POSITION), 2088);
  for (int i = 0; i < 10; ++i) {
    sim.step(0.01, 4000.0);
  }
  EXPECT_EQ(sim.reg_u16(1, reg::PRESENT_POSITION), 2148);
}
