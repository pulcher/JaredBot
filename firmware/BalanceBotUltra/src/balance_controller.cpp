#include "balance_controller.h"

void BalanceController::setSettings(const ControlSettings &settings)
{
    settings_ = settings;
}

const ControlSettings &BalanceController::settings() const
{
    return settings_;
}

void BalanceController::setCalibratedZero(float zeroDeg)
{
    calibratedZeroDeg_ = zeroDeg;
}

float BalanceController::calibratedZero() const
{
    return calibratedZeroDeg_;
}

void BalanceController::arm(long currentAverageCount)
{
    armed_ = true;
    balanceIntegral_ = 0.0f;
    holdIntegral_ = 0.0f;
    holdTargetCount_ = (float)currentAverageCount;
}

void BalanceController::disarm()
{
    armed_ = false;
    balanceIntegral_ = 0.0f;
    holdIntegral_ = 0.0f;
}

bool BalanceController::armed() const
{
    return armed_;
}

void BalanceController::reset()
{
    balanceIntegral_ = 0.0f;
    holdIntegral_ = 0.0f;
}

ControlOutput BalanceController::update(float dtSeconds, float angleDeg, float angleRateDegPerSec,
                                        long currentAverageCount, long deltaAverageCount)
{
    ControlOutput output;

    if (!armed_ || dtSeconds <= 0.0f) {
        return output;
    }

    const float velocityCountsPerSec = (float)deltaAverageCount / dtSeconds;
    const bool settled = fabs(angleDeg - calibratedZeroDeg_) < hw::kSettledAngleDeg &&
                         fabs(velocityCountsPerSec) < hw::kSettledVelocityCountsPerSec;

    if (settled) {
        holdTargetCount_ = (1.0f - hw::kHoldTargetBlend) * holdTargetCount_ +
                           hw::kHoldTargetBlend * (float)currentAverageCount;
    }

    const float positionErrorCounts = holdTargetCount_ - (float)currentAverageCount;
    holdIntegral_ = clampf(holdIntegral_ + positionErrorCounts * dtSeconds, -5000.0f, 5000.0f);

    const float holdBiasDeg = clampf(
        settings_.holdKp * positionErrorCounts +
        settings_.holdKi * holdIntegral_ -
        settings_.velocityKp * velocityCountsPerSec,
        -hw::kMaxHoldBiasDeg,
        hw::kMaxHoldBiasDeg);

    const float targetAngleDeg = calibratedZeroDeg_ + settings_.trimDeg + holdBiasDeg;
    const float balanceErrorDeg = angleDeg - targetAngleDeg;

    if (settings_.ki != 0.0f) {
        balanceIntegral_ = clampf(balanceIntegral_ + balanceErrorDeg * dtSeconds, -300.0f, 300.0f);
    } else {
        balanceIntegral_ = 0.0f;
    }

    const float pdOutput =
        settings_.kp * balanceErrorDeg +
        settings_.ki * balanceIntegral_ +
        settings_.kd * angleRateDegPerSec;

    const int16_t pwm = (int16_t)(-pdOutput);
    output.leftPwm = pwm;
    output.rightPwm = pwm;
    output.targetAngleDeg = targetAngleDeg;
    output.holdBiasDeg = holdBiasDeg;
    output.positionErrorCounts = positionErrorCounts;
    output.velocityCountsPerSec = velocityCountsPerSec;
    output.balanceErrorDeg = balanceErrorDeg;
    return output;
}

float BalanceController::clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}
