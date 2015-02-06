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
    class Instruments
    {
    public:
        Instruments(const AP_HAL::HAL& hal);

        Vector3f GetGyro();
        Vector3f GetOrientation();

        void Update();

    private:
        AP_InertialSensor_MPU6000 ins;
        const AP_HAL::HAL& hal;
    };
}
