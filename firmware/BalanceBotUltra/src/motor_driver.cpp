#include "motor_driver.h"

#include "hardware_config.h"

namespace {

static int16_t clampPwm(int16_t pwm)
{
    if (pwm > 255) return 255;
    if (pwm < -255) return -255;
    return pwm;
}

} // namespace

void MotorDriver::begin()
{
    pinMode(hw::kRightMotorIn1, OUTPUT);
    pinMode(hw::kRightMotorIn2, OUTPUT);
    pinMode(hw::kRightMotorPwm, OUTPUT);
    pinMode(hw::kLeftMotorIn1, OUTPUT);
    pinMode(hw::kLeftMotorIn2, OUTPUT);
    pinMode(hw::kLeftMotorPwm, OUTPUT);
    stop();
}

void MotorDriver::stop()
{
    analogWrite(hw::kLeftMotorPwm, 0);
    analogWrite(hw::kRightMotorPwm, 0);
    digitalWrite(hw::kLeftMotorIn1, LOW);
    digitalWrite(hw::kLeftMotorIn2, LOW);
    digitalWrite(hw::kRightMotorIn1, LOW);
    digitalWrite(hw::kRightMotorIn2, LOW);
}

void MotorDriver::setMinPwm(uint8_t minPwm)
{
    minPwm_ = minPwm;
}

uint8_t MotorDriver::minPwm() const
{
    return minPwm_;
}

void MotorDriver::apply(int16_t leftPwm, int16_t rightPwm)
{
    applySide(hw::kLeftMotorIn1, hw::kLeftMotorIn2, hw::kLeftMotorPwm, leftPwm, hw::kInvertLeftMotor);
    applySide(hw::kRightMotorIn1, hw::kRightMotorIn2, hw::kRightMotorPwm, rightPwm, hw::kInvertRightMotor);
}

void MotorDriver::applySide(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t pwm, bool invert)
{
    pwm = clampPwm(pwm);
    if (invert) {
        pwm = -pwm;
    }

    if (pwm > 0 && pwm < minPwm_) pwm = minPwm_;
    if (pwm < 0 && -pwm < minPwm_) pwm = -minPwm_;

    if (pwm == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, 0);
        return;
    }

    if (pwm > 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwmPin, pwm);
    } else {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, -pwm);
    }
}
