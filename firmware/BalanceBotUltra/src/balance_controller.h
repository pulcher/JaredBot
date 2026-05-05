#pragma once

#include <Arduino.h>

#include "hardware_config.h"
#include "settings.h"

struct ControlOutput
{
    int16_t leftPwm = 0;
    int16_t rightPwm = 0;
    float targetAngleDeg = 0.0f;
    float holdBiasDeg = 0.0f;
    float positionErrorCounts = 0.0f;
    float velocityCountsPerSec = 0.0f;
    float balanceErrorDeg = 0.0f;
};

class BalanceController
{
public:
    void setSettings(const ControlSettings &settings);
    const ControlSettings &settings() const;

    void setCalibratedZero(float zeroDeg);
    float calibratedZero() const;

    void arm(long currentAverageCount);
    void disarm();
    bool armed() const;
    void reset();

    ControlOutput update(float dtSeconds, float angleDeg, float angleRateDegPerSec,
                         long currentAverageCount, long deltaAverageCount);

private:
    static float clampf(float value, float low, float high);

    ControlSettings settings_;
    float calibratedZeroDeg_ = 0.0f;
    float balanceIntegral_ = 0.0f;
    float holdIntegral_ = 0.0f;
    float holdTargetCount_ = 0.0f;
    bool armed_ = false;
};
