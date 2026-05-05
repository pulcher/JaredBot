#include "imu_filter.h"

#include <Wire.h>

void ImuFilter::begin()
{
    Wire.begin();
    mpu_.initialize();
    delay(2);
    resetFilterState();
}

float ImuFilter::calibrateBaseline(uint16_t samples, uint16_t sampleDelayMs)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; ++i) {
        mpu_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
        const float angle = -atan2((float)ay_, (float)az_) * (180.0f / PI);
        sum += angle;
        delay(sampleDelayMs);
    }
    return sum / (float)samples;
}

bool ImuFilter::update(float dtSeconds)
{
    if (dtSeconds <= 0.0f) {
        return false;
    }

    mpu_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);

    const float measuredAngle = -atan2((float)ay_, (float)az_) * (180.0f / PI);
    const float gyroX = -(float)gx_ / 131.0f;
    const float gyroY = -(float)gy_ / 131.0f;
    yawRateDegPerSec_ = -(float)gz_ / 131.0f;

    angleDeg_ += (gyroX - qBias_) * dtSeconds;
    angleError_ = measuredAngle - angleDeg_;

    pDot_[0] = qAngle_ - p_[0][1] - p_[1][0];
    pDot_[1] = -p_[1][1];
    pDot_[2] = -p_[1][1];
    pDot_[3] = qGyro_;

    p_[0][0] += pDot_[0] * dtSeconds;
    p_[0][1] += pDot_[1] * dtSeconds;
    p_[1][0] += pDot_[2] * dtSeconds;
    p_[1][1] += pDot_[3] * dtSeconds;

    const float pCt0 = p_[0][0];
    const float pCt1 = p_[1][0];
    const float e = rAngle_ + pCt0;
    const float k0 = pCt0 / e;
    const float k1 = pCt1 / e;
    const float t0 = pCt0;
    const float t1 = p_[0][1];

    p_[0][0] -= k0 * t0;
    p_[0][1] -= k0 * t1;
    p_[1][0] -= k1 * t0;
    p_[1][1] -= k1 * t1;

    qBias_ += k1 * angleError_;
    angleRateDegPerSec_ = gyroX - qBias_;
    angleDeg_ += k0 * angleError_;

    const float auxMeasuredAngle = -atan2((float)ax_, (float)az_) * (180.0f / PI);
    auxAngleDeg_ = k1_ * auxMeasuredAngle + (1.0f - k1_) * (auxAngleDeg_ + gyroY * dtSeconds);
    return true;
}

void ImuFilter::setKalmanTuning(float qAngle, float qGyro, float rAngle, float k1)
{
    qAngle_ = qAngle;
    qGyro_ = qGyro;
    rAngle_ = rAngle;
    k1_ = k1;
}

void ImuFilter::resetFilterState()
{
    qBias_ = 0.0f;
    angleError_ = 0.0f;
    angleDeg_ = 0.0f;
    angleRateDegPerSec_ = 0.0f;
    yawRateDegPerSec_ = 0.0f;
    auxAngleDeg_ = 0.0f;
    pDot_[0] = pDot_[1] = pDot_[2] = pDot_[3] = 0.0f;
    p_[0][0] = 1.0f;
    p_[0][1] = 0.0f;
    p_[1][0] = 0.0f;
    p_[1][1] = 1.0f;
}

float ImuFilter::angleDeg() const
{
    return angleDeg_;
}

float ImuFilter::angleRateDegPerSec() const
{
    return angleRateDegPerSec_;
}

float ImuFilter::yawRateDegPerSec() const
{
    return yawRateDegPerSec_;
}

float ImuFilter::auxAngleDeg() const
{
    return auxAngleDeg_;
}
