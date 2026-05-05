#include "telemetry.h"

Telemetry::Telemetry(Print &usb, Print &relay)
    : usb_(usb), relay_(relay)
{
}

void Telemetry::sendBoot(const __FlashStringHelper *message)
{
    usb_.print(F("[BOOT] "));
    usb_.println(message);

    relay_.print(F("V:BOOT,"));
    relay_.println(message);
}

void Telemetry::sendEvent(const __FlashStringHelper *eventName, const String &detail)
{
    usb_.print(F("[EVENT] "));
    usb_.print(eventName);
    usb_.print(F(" "));
    usb_.println(detail);

    relay_.print(F("F:"));
    relay_.print(eventName);
    relay_.print(F(","));
    relay_.println(detail);
}

void Telemetry::sendState(hw::SystemMode mode, bool armed, const String &faultName)
{
    printStateFrame(usb_, mode, armed, faultName);
    printStateFrame(relay_, mode, armed, faultName);
}

void Telemetry::sendSettings(const ControlSettings &settings, float calibratedZeroDeg)
{
    printSettingsFrame(usb_, settings, calibratedZeroDeg);
    printSettingsFrame(relay_, settings, calibratedZeroDeg);
}

void Telemetry::sendRuntime(float angleDeg, float angleRateDegPerSec, float yawRateDegPerSec,
                            const ControlOutput &output, uint32_t dtLastUs, uint32_t dtMinUs,
                            uint32_t dtMaxUs, uint32_t dtAvgUs, uint16_t dtCount,
                            bool toUsb, bool toRelay)
{
    if (toUsb) {
        printRuntimeFrame(usb_, angleDeg, angleRateDegPerSec, yawRateDegPerSec, output,
                          dtLastUs, dtMinUs, dtMaxUs, dtAvgUs, dtCount);
    }
    if (toRelay) {
        printRuntimeFrame(relay_, angleDeg, angleRateDegPerSec, yawRateDegPerSec, output,
                          dtLastUs, dtMinUs, dtMaxUs, dtAvgUs, dtCount);
    }
}

void Telemetry::sendEncoders(long left, long right, long deltaLeft, long deltaRight,
                             bool toUsb, bool toRelay)
{
    if (toUsb) {
        printEncoderFrame(usb_, left, right, deltaLeft, deltaRight);
    }
    if (toRelay) {
        printEncoderFrame(relay_, left, right, deltaLeft, deltaRight);
    }
}

const __FlashStringHelper *Telemetry::modeName(hw::SystemMode mode) const
{
    switch (mode) {
        case hw::SystemMode::Boot: return F("BOOT");
        case hw::SystemMode::Calibrating: return F("CAL");
        case hw::SystemMode::Idle: return F("IDLE");
        case hw::SystemMode::Balance: return F("BAL");
        case hw::SystemMode::TestImu: return F("TEST_IMU");
        case hw::SystemMode::TestMotors: return F("TEST_MOTORS");
        case hw::SystemMode::TestEncoders: return F("TEST_ENCODERS");
        case hw::SystemMode::TestSerial: return F("TEST_SERIAL");
        case hw::SystemMode::Fault: return F("FAULT");
    }
    return F("UNKNOWN");
}

void Telemetry::printStateFrame(Print &out, hw::SystemMode mode, bool armed, const String &faultName)
{
    out.print(F("V:"));
    out.print(modeName(mode));
    out.print(F(","));
    out.print(armed ? 1 : 0);
    out.print(F(","));
    out.println(faultName);
}

void Telemetry::printSettingsFrame(Print &out, const ControlSettings &settings, float calibratedZeroDeg)
{
    out.print(F("S:"));
    out.print(settings.trimDeg, 4);
    out.print(F(","));
    out.print(settings.kp, 4);
    out.print(F(","));
    out.print(settings.ki, 4);
    out.print(F(","));
    out.print(settings.kd, 4);
    out.print(F(","));
    out.print(settings.holdKp, 4);
    out.print(F(","));
    out.print(settings.holdKi, 4);
    out.print(F(","));
    out.print(settings.velocityKp, 4);
    out.print(F(","));
    out.print(settings.qAngle, 6);
    out.print(F(","));
    out.print(settings.qGyro, 6);
    out.print(F(","));
    out.print(settings.rAngle, 6);
    out.print(F(","));
    out.print(settings.k1, 4);
    out.print(F(","));
    out.print(settings.minPwm);
    out.print(F(","));
    out.println(calibratedZeroDeg, 4);
}

void Telemetry::printRuntimeFrame(Print &out, float angleDeg, float angleRateDegPerSec, float yawRateDegPerSec,
                                  const ControlOutput &output, uint32_t dtLastUs, uint32_t dtMinUs,
                                  uint32_t dtMaxUs, uint32_t dtAvgUs, uint16_t dtCount)
{
    out.print(F("T:"));
    out.print(angleDeg, 3);
    out.print(F(","));
    out.print(angleRateDegPerSec, 3);
    out.print(F(","));
    out.print(yawRateDegPerSec, 3);
    out.print(F(","));
    out.print(output.targetAngleDeg, 3);
    out.print(F(","));
    out.print(output.holdBiasDeg, 3);
    out.print(F(","));
    out.print(output.positionErrorCounts, 2);
    out.print(F(","));
    out.print(output.velocityCountsPerSec, 2);
    out.print(F(","));
    out.print(output.leftPwm);
    out.print(F(","));
    out.print(output.rightPwm);
    out.print(F(","));
    out.print(dtLastUs);
    out.print(F(","));
    out.print(dtMinUs);
    out.print(F(","));
    out.print(dtMaxUs);
    out.print(F(","));
    out.print(dtAvgUs);
    out.print(F(","));
    out.println(dtCount);
}

void Telemetry::printEncoderFrame(Print &out, long left, long right, long deltaLeft, long deltaRight)
{
    out.print(F("E:"));
    out.print(left);
    out.print(F(","));
    out.print(right);
    out.print(F(","));
    out.print(deltaLeft);
    out.print(F(","));
    out.println(deltaRight);
}
