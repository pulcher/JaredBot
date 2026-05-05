#pragma once

#include <Arduino.h>

namespace hw {

enum class BringupMode : uint8_t {
    Normal = 0,
    TestImu,
    TestMotors,
    TestEncoders,
    TestSerial,
};

enum class SystemMode : uint8_t {
    Boot = 0,
    Calibrating,
    Idle,
    Balance,
    TestImu,
    TestMotors,
    TestEncoders,
    TestSerial,
    Fault,
};

static constexpr BringupMode kBringupMode = BringupMode::Normal;
static constexpr bool kAutoArmAfterCalibration = false;

static constexpr uint8_t kRightMotorIn1 = 8;
static constexpr uint8_t kRightMotorIn2 = 12;
static constexpr uint8_t kRightMotorPwm = 10;

static constexpr uint8_t kLeftMotorIn1 = 7;
static constexpr uint8_t kLeftMotorIn2 = 6;
static constexpr uint8_t kLeftMotorPwm = 9;

static constexpr uint8_t kRightEncoderPin = 5;
static constexpr uint8_t kLeftEncoderPin = 4;

static constexpr uint8_t kRelayRxPin = 17; // A3
static constexpr uint8_t kRelayTxPin = 16; // A2

static constexpr uint8_t kBuzzerPin = 11;
static constexpr uint8_t kButtonPin = 13;

static constexpr bool kInvertLeftMotor = false;
static constexpr bool kInvertRightMotor = false;
static constexpr bool kInvertLeftEncoder = false;
static constexpr bool kInvertRightEncoder = false;

static constexpr unsigned long kUsbBaud = 115200;
static constexpr unsigned long kRelayBaud = 9600;

static constexpr uint32_t kControlPeriodUs = 5000;
static constexpr uint16_t kCalibrationSamples = 400;
static constexpr uint16_t kCalibrationDelayMs = 5;

static constexpr unsigned long kUsbRuntimeTelemetryMs = 100;
static constexpr unsigned long kUsbEncoderTelemetryMs = 250;
static constexpr unsigned long kRelayRuntimeTelemetryMs = 500;
static constexpr unsigned long kRelayEncoderTelemetryMs = 500;
static constexpr unsigned long kSettingsTelemetryMs = 15000;
static constexpr unsigned long kStateTelemetryMs = 1000;

static constexpr float kTiltFaultDeg = 45.0f;
static constexpr float kHoldTargetBlend = 0.02f;
static constexpr float kSettledAngleDeg = 2.5f;
static constexpr float kSettledVelocityCountsPerSec = 18.0f;
static constexpr float kMaxHoldBiasDeg = 6.0f;

} // namespace hw
