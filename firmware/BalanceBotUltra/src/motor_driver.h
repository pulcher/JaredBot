#pragma once

#include <Arduino.h>

class MotorDriver
{
public:
    void begin();
    void stop();
    void setMinPwm(uint8_t minPwm);
    uint8_t minPwm() const;
    void apply(int16_t leftPwm, int16_t rightPwm);

private:
    uint8_t minPwm_ = 0;
    void applySide(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t pwm, bool invert);
};
