#pragma once

#include <Arduino.h>
#include <stdint.h>

// Arduino Uno (ATmega328P) does not support ESP32-style NVS.
// This module persists settings to the internal EEPROM with CRC + A/B slots.

struct PersistedSettings
{
    float trimDeg;   // offset added after IMU auto-calibration

    float kp;
    float ki;
    float kd;

    float Q_angle;
    float Q_gyro;
    float R_angle;
    float K1;
};

// Loads newest valid record from EEPROM.
// Returns true if settings were loaded; false if EEPROM had no valid record.
// On success, seqOut/slotOut are filled for subsequent saves.
bool loadPersistedSettings(PersistedSettings &out, uint32_t &seqOut, uint8_t &slotOut);

// Saves settings to EEPROM using A/B slot scheme.
// seqInOut increments on successful save; slotInOut toggles to the written slot.
// Returns true on success.
bool savePersistedSettings(const PersistedSettings &in, uint32_t &seqInOut, uint8_t &slotInOut);
