#include "encoder_service.h"

#include "hardware_config.h"

void EncoderService::begin()
{
    pinMode(hw::kRightEncoderPin, INPUT_PULLUP);
    pinMode(hw::kLeftEncoderPin, INPUT_PULLUP);
    lastPortD_ = PIND;
}

void EncoderService::poll()
{
    const uint8_t nowPortD = PIND;
    const uint8_t changed = nowPortD ^ lastPortD_;
    lastPortD_ = nowPortD;

    if (changed & _BV(hw::kRightEncoderPin)) {
        if (nowPortD & _BV(hw::kRightEncoderPin)) {
            rightCount_ += (hw::kInvertRightEncoder ? -1 : 1);
        }
    }

    if (changed & _BV(hw::kLeftEncoderPin)) {
        if (nowPortD & _BV(hw::kLeftEncoderPin)) {
            leftCount_ += (hw::kInvertLeftEncoder ? -1 : 1);
        }
    }
}

void EncoderService::snapshot(long &left, long &right) const
{
    noInterrupts();
    left = leftCount_;
    right = rightCount_;
    interrupts();
}

void EncoderService::reset()
{
    noInterrupts();
    leftCount_ = 0;
    rightCount_ = 0;
    interrupts();
}
