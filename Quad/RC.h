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
    class RC
    {
    public:
        RC(const AP_HAL::HAL& hal);

        long GetThrottle();
        long GetYaw();
        long GetPitch();
        long GetRoll();
        void Read();

    private:
        uint16_t channels[8];
        const AP_HAL::HAL& hal;
    };
}
