#pragma once

#include <Arduino.h>
#include <MPU6050.h>

class ImuFilter
{
public:
    void begin();
    float calibrateBaseline(uint16_t samples, uint16_t sampleDelayMs);
    bool update(float dtSeconds);
    void setKalmanTuning(float qAngle, float qGyro, float rAngle, float k1);
    void resetFilterState();

    float angleDeg() const;
    float angleRateDegPerSec() const;
    float yawRateDegPerSec() const;
    float auxAngleDeg() const;

private:
    MPU6050 mpu_;
    int16_t ax_ = 0;
    int16_t ay_ = 0;
    int16_t az_ = 0;
    int16_t gx_ = 0;
    int16_t gy_ = 0;
    int16_t gz_ = 0;

    float angleDeg_ = 0.0f;
    float angleRateDegPerSec_ = 0.0f;
    float yawRateDegPerSec_ = 0.0f;
    float auxAngleDeg_ = 0.0f;

    float qAngle_ = 0.001f;
    float qGyro_ = 0.003f;
    float rAngle_ = 0.5f;
    float k1_ = 0.05f;

    float qBias_ = 0.0f;
    float angleError_ = 0.0f;
    float pDot_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float p_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
};
