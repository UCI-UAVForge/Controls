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

#include "Instruments.h"

const float MAG_DECLINATION = 12.08; // 12.08 for UCI as of 2015

namespace Quad
{
    Instruments::Instruments(const AP_HAL::HAL& hal)
        : hal(hal), baro(&AP_Baro_MS5611::spi), ahrs(&ins, (GPS*&)gps)
    {
        // we need to stop the barometer from holding the SPI bus
        // TODO: Does this stop the barometer from working?
        hal.gpio->pinMode(40, GPIO_OUTPUT);
        hal.gpio->write(40, 1);

        // Turn on MPU6000 - quad must be kept still as gyros will calibrate
        ins.init(AP_InertialSensor::COLD_START,
            AP_InertialSensor::RATE_100HZ,
            NULL);

        // initialise sensor fusion on MPU6000 chip (aka DigitalMotionProcessing/DMP)
        hal.scheduler->suspend_timer_procs();  // stop bus collisions
        ins.dmp_init();
        hal.scheduler->resume_timer_procs();

        // Initialize compass
        compass.init();
        //compass.set_declination(ToRad(MAG_DECLINATION));

        // Initialize Barometer
        baro.init();
        baro.calibrate();

        // Initialize ahrs
        ahrs.init();
        ahrs.set_compass(&compass);

        // Warmup and accumulate error...
        for (int i = 0; i < 1600; i++)
        {
            while (ins.num_samples_available() == 0);

            ins.update();
            compass.accumulate();

            float roll;
            float pitch;
            float yaw;
            ins.quaternion.to_euler(&roll, &pitch, &yaw);
            attitudeOffset = Vector3f(ToDeg(roll), ToDeg(pitch), ToDeg(yaw));
        }
    }

    void Instruments::Update()
    {
        // Wait until new orientation data (normally 5ms max)
        while (ins.num_samples_available() == 0);

        ahrs.update();
        ins.update();
        compass.accumulate();
        baro.accumulate();
    }

    Vector3f Instruments::GetAcceleration()
    {
        return Vector3f(0, 0, 0);
    }

    Vector3f Instruments::GetVelocity()
    {
        return Vector3f(0, 0, 0);
    }

    Vector3f Instruments::GetPosition()
    {
        baro.read();
        return Vector3f(0, 0, baro.get_altitude());
    }

    Vector3f Instruments::GetGyro()
    {
        Vector3f rad = ins.get_gyro();
        return Vector3f(ToDeg(rad.x), ToDeg(rad.y), ToDeg(rad.z));
    }
    
    Vector3f Instruments::GetAttitude()
    {
        float roll;
        float pitch;
        float yaw;
        ins.quaternion.to_euler(&roll, &pitch, &yaw);
        return Vector3f(ToDeg(roll), ToDeg(pitch), ToDeg(yaw)) - attitudeOffset;
    }

    float Instruments::GetHeading()
    {
        compass.read();
        float roll;
        float pitch;
        float yaw;
        ins.quaternion.to_euler(&roll, &pitch, &yaw);
        return ToDeg(compass.calculate_heading(roll, pitch));
    }
}
