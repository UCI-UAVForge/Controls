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
        : hal(hal), baro(&AP_Baro_MS5611::spi), ahrs(&ins, (GPS*&)gps), nav(&ahrs, &ins, &baro, (GPS**)&gps)
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

        // Set ahrs compass
        ahrs.set_compass(&compass);

        // Initialize inertial nav
        nav.init();
        nav.set_velocity_xy(0, 0);
        nav.set_current_position(0, 0);

        // Initialize GPS
#ifdef ENABLE_GPS
        gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
#endif
    }

    void Instruments::Update()
    {
        // Wait until new orientation data (normally 5ms max)
        while (ins.num_samples_available() == 0);

        uint32_t now = hal.scheduler->micros();
        // Note: ins will be updated by ahrs automatically
        //ins.update();
        compass.accumulate();
        compass.read();
        baro.accumulate();
        ahrs.update();
        nav.update((now - lastUpdate) / 1000000.0F);
        lastUpdate = now;
    }

    Vector3f Instruments::GetAcceleration()
    {
        return ahrs.get_accel_ef();
    }

    Vector3f Instruments::GetVelocity()
    {
        return nav.get_velocity() * 100;
    }

    Vector3f Instruments::GetPosition()
    {
        return nav.get_position() * 100;
    }

    Vector3f Instruments::GetGyro()
    {
        Vector3f rad = ahrs.get_gyro();
        return Vector3f(ToDeg(rad.x), ToDeg(rad.y), ToDeg(rad.z));
    }
    
    Vector3f Instruments::GetAttitude()
    {
        Matrix3f dcm = ahrs.get_dcm_matrix();
        float roll;
        float pitch;
        float yaw;
        dcm.to_euler(&roll, &pitch, &yaw);
        return Vector3f(ToDeg(roll), ToDeg(pitch), ToDeg(yaw));
    }

    float Instruments::GetHeading()
    {
        Vector3f a = GetAttitude();
        return compass.calculate_heading(a.x, a.y);
    }
}
