#pragma once

#include <Arduino.h>

struct ControlSettings
{
    float trimDeg = 0.0f;

    float kp = 34.0f;
    float ki = 0.02f;
    float kd = 0.90f;

    float holdKp = 0.0f;
    float holdKi = 0.0f;
    float velocityKp = 0.0f;

    float qAngle = 0.001f;
    float qGyro = 0.003f;
    float rAngle = 0.5f;
    float k1 = 0.05f;

    uint8_t minPwm = 0;
    uint8_t reserved0 = 0;
    uint8_t reserved1 = 0;
    uint8_t reserved2 = 0;
};
