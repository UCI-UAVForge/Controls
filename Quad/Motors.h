#pragma once

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

namespace Quad
{
    class Motors
    {
    public:
        Motors(const AP_HAL::HAL& hal);

        void SetFrontLeft(long throttle);
        void SetFrontRight(long throttle);
        void SetBackLeft(long throttle);
        void SetBackRight(long throttle);
        void Stop();

    private:
        const AP_HAL::HAL& hal;
    };
}
