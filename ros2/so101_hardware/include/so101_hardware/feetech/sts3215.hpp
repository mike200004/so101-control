// STS3215 control table (memory map). Addresses match the Feetech STS series
// datasheet and LeRobot's `STS_SMS_SERIES_CONTROL_TABLE`.
#pragma once

#include <cstdint>

namespace so101_hardware::feetech::sts3215
{

constexpr uint16_t kModelNumber = 777;  // value of register MODEL_NUMBER
constexpr int kResolution = 4096;       // encoder steps per revolution

// EEPROM area (persistent; writes require LOCK = 0)
constexpr uint8_t MODEL_NUMBER = 3;          // u16
constexpr uint8_t ID = 5;                    // u8
constexpr uint8_t BAUD_RATE = 6;             // u8
constexpr uint8_t RETURN_DELAY_TIME = 7;     // u8, 2 us units
constexpr uint8_t MIN_POSITION_LIMIT = 9;    // u16
constexpr uint8_t MAX_POSITION_LIMIT = 11;   // u16
constexpr uint8_t P_COEFFICIENT = 21;        // u8
constexpr uint8_t D_COEFFICIENT = 22;        // u8
constexpr uint8_t I_COEFFICIENT = 23;        // u8
constexpr uint8_t HOMING_OFFSET = 31;        // u16, sign-magnitude bit 11
constexpr uint8_t OPERATING_MODE = 33;       // u8, 0 = position

// RAM area
constexpr uint8_t TORQUE_ENABLE = 40;        // u8
constexpr uint8_t ACCELERATION = 41;         // u8
constexpr uint8_t GOAL_POSITION = 42;        // u16, sign-magnitude bit 15
constexpr uint8_t GOAL_TIME = 44;            // u16
constexpr uint8_t GOAL_VELOCITY = 46;        // u16, sign-magnitude bit 15
constexpr uint8_t TORQUE_LIMIT = 48;         // u16
constexpr uint8_t LOCK = 55;                 // u8, 1 = EEPROM write-protected
constexpr uint8_t PRESENT_POSITION = 56;     // u16, sign-magnitude bit 15
constexpr uint8_t PRESENT_VELOCITY = 58;     // u16, sign-magnitude bit 15, steps/s
constexpr uint8_t PRESENT_LOAD = 60;         // u16, sign-magnitude bit 10
constexpr uint8_t PRESENT_VOLTAGE = 62;      // u8, 0.1 V
constexpr uint8_t PRESENT_TEMPERATURE = 63;  // u8, deg C
constexpr uint8_t STATUS = 65;               // u8
constexpr uint8_t MOVING = 66;               // u8
constexpr uint8_t PRESENT_CURRENT = 69;      // u16

constexpr int kPositionSignBit = 15;
constexpr int kVelocitySignBit = 15;
constexpr int kLoadSignBit = 10;
constexpr int kHomingOffsetSignBit = 11;

}  // namespace so101_hardware::feetech::sts3215
