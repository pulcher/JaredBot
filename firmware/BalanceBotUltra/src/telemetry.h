#pragma once

#include <Arduino.h>

#include "balance_controller.h"
#include "hardware_config.h"
#include "settings.h"

class Telemetry
{
public:
    Telemetry(Print &usb, Print &relay);

    void sendBoot(const __FlashStringHelper *message);
    void sendEvent(const __FlashStringHelper *eventName, const String &detail);
    void sendState(hw::SystemMode mode, bool armed, const String &faultName);
    void sendSettings(const ControlSettings &settings, float calibratedZeroDeg);
    void sendRuntime(float angleDeg, float angleRateDegPerSec, float yawRateDegPerSec,
                     const ControlOutput &output, uint32_t dtLastUs, uint32_t dtMinUs,
                     uint32_t dtMaxUs, uint32_t dtAvgUs, uint16_t dtCount,
                     bool toUsb, bool toRelay);
    void sendEncoders(long left, long right, long deltaLeft, long deltaRight,
                      bool toUsb, bool toRelay);

private:
    Print &usb_;
    Print &relay_;
    const __FlashStringHelper *modeName(hw::SystemMode mode) const;
    void printStateFrame(Print &out, hw::SystemMode mode, bool armed, const String &faultName);
    void printSettingsFrame(Print &out, const ControlSettings &settings, float calibratedZeroDeg);
    void printRuntimeFrame(Print &out, float angleDeg, float angleRateDegPerSec, float yawRateDegPerSec,
                           const ControlOutput &output, uint32_t dtLastUs, uint32_t dtMinUs,
                           uint32_t dtMaxUs, uint32_t dtAvgUs, uint16_t dtCount);
    void printEncoderFrame(Print &out, long left, long right, long deltaLeft, long deltaRight);
};
