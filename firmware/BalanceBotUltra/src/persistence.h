#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "settings.h"

bool loadPersistedSettings(ControlSettings &out, uint32_t &seqOut, uint8_t &slotOut);
bool savePersistedSettings(const ControlSettings &in, uint32_t &seqInOut, uint8_t &slotInOut);
