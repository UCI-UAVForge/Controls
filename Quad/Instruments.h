// Copyright 2015 Jason Watkins & Gareth Owen
// This file is part of the UCI UAVForge Quad-copter Controls System. It is
// based on the source code provided at 
// https://ghowen.me/build-your-own-quadcopter-autopilot/
//
// UCI UAVForge Quad-copter Controls System is free software : you can
// redistribute it and / or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// UCI UAVForge Quad-copter Controls System is distributed in the hope that it
// will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// UCI UAVForge Quad-copter Controls System. If not, see
// <http://www.gnu.org/licenses/>.

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
