// Copyright 2015 Jason Watkins
// This file is part of the UCI UAVForge Quad-copter Controls System.
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

//#define ENABLE_GPS

#include <AP_Compass.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_GPS.h>
#include <AP_HAL.h>

namespace Quad
{
    class Instruments
    {
    public:
        Instruments(const AP_HAL::HAL& hal);

        Vector3f GetAcceleration();
        Vector3f GetVelocity();
        Vector3f GetPosition();

        Vector3f GetGyro();
        Vector3f GetAttitude();
        float GetHeading();

        void Update();

    private:
        const AP_HAL::HAL& hal;

        AP_InertialSensor_MPU6000 ins;
        AP_Compass_HMC5843 compass;
        AP_Baro_MS5611 baro;
#ifdef ENABLE_GPS
        AP_GPS_UBLOX* gps;
#else
        GPS *gps;
#endif

        AP_AHRS_MPU6000 ahrs;

        Vector3f attitudeOffset;
        float headingOffset;

        uint32_t lastUpdate;
    };
}
