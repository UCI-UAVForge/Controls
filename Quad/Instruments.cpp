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

#include "Instruments.h"

namespace Quad
{
    Instruments::Instruments(const AP_HAL::HAL& hal)
        : hal(hal)
    {
        // Turn off Barometer to avoid bus collisions
        hal.gpio->pinMode(40, GPIO_OUTPUT);
        hal.gpio->write(40, 1);

        // Turn on MPU6050 - quad must be kept still as gyros will calibrate
        this->ins.init(AP_InertialSensor::COLD_START,
            AP_InertialSensor::RATE_100HZ,
            NULL);

        // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
        hal.scheduler->suspend_timer_procs();  // stop bus collisions
        ins.dmp_init();
        hal.scheduler->resume_timer_procs();
    }

    void Instruments::Update()
    {
        // Wait until new orientation data (normally 5ms max)
        while (ins.num_samples_available() == 0);
        ins.update();
    }

    Vector3f Instruments::GetVelocity()
    {
        Vector3f orientation = GetOrientation();
        Vector3f accel = ins.get_accel();
    }

    Vector3f Instruments::GetGyro()
    {
        Vector3f rad = ins.get_gyro();
        return Vector3f(ToDeg(rad.x), ToDeg(rad.y), ToDeg(rad.z));
    }


    Vector3f Instruments::GetOrientation()
    {
        float roll;
        float pitch;
        float yaw;
        ins.quaternion.to_euler(&roll, &pitch, &yaw);
        roll = ToDeg(roll);
        pitch = ToDeg(pitch);
        yaw = ToDeg(yaw);
        return Vector3f(roll, pitch, yaw);
    }
}
