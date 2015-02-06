#include "Motors.h"

const int FL = 2;
const int FR = 0;
const int BL = 1;
const int BR = 3;

namespace Quad
{
    Motors::Motors(const AP_HAL::HAL& hal)
        : hal(hal)
    {
        // Enable the motors and set at 490Hz update
        hal.rcout->set_freq(0xF, 490);
        hal.rcout->enable_mask(0xFF);
    }

    void Motors::SetFrontLeft(long throttle)
    {
        hal.rcout->write(FL, throttle);
    }

    void Motors::SetFrontRight(long throttle)
    {
        hal.rcout->write(FR, throttle);
    }

    void Motors::SetBackLeft(long throttle)
    {
        hal.rcout->write(BL, throttle);
    }

    void Motors::SetBackRight(long throttle)
    {
        hal.rcout->write(BR, throttle);
    }

    void Motors::Stop()
    {
        SetFrontLeft(1000);
        SetFrontRight(1000);
        SetBackLeft(1000);
        SetBackRight(1000);
    }
}
