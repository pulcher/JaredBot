#pragma once

#include <Arduino.h>

class EncoderService
{
public:
    void begin();
    void poll();
    void snapshot(long &left, long &right) const;
    void reset();

private:
    volatile long leftCount_ = 0;
    volatile long rightCount_ = 0;
    uint8_t lastPortD_ = 0;
};
