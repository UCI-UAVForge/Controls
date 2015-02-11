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


#define TELEM

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

#include "Motors.h"
#include "Instruments.h"
#include "RC.h"
#include "RotationRateControl.h"
#include "AttitudeControl.h"

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

int main(void)
{
    hal.init(0, NULL);

    // Initialize LED's
    AP_HAL::DigitalSource* b_led;
    AP_HAL::DigitalSource* y_led;
    AP_HAL::DigitalSource* r_led;
    b_led = hal.gpio->channel(25);
    y_led = hal.gpio->channel(26);
    r_led = hal.gpio->channel(27);
    b_led->mode(GPIO_OUTPUT);
    y_led->mode(GPIO_OUTPUT);
    r_led->mode(GPIO_OUTPUT);

    // Yellow LED on indicates we are in startup
    b_led->write(1);
    y_led->write(0);
    r_led->write(1);

    Quad::Motors motors = Quad::Motors(hal);
    Quad::Instruments ins = Quad::Instruments(hal);
    Quad::RC rc = Quad::RC(hal);

    Quad::RotationRateControl rrc;
    Quad::AttitudeControl ac;
    

    hal.scheduler->system_initialized();

    // Red LED on indicates we are live
    y_led->write(1);
    r_led->write(0);
    for (;;)
    {
        ins.Update();

        // Read RC transmitter and map to sensible values
        rc.Read();
        long rcthr = rc.GetThrottle();
        Vector3f attitudeTargets = rc.GetAttitudeInputs();

        // Ask MPU6050 for orientation
        Vector3f attitude = ins.GetAttitude();

        // Ask MPU6050 for gyro data
        Vector3f gyro = ins.GetGyro();

        // Do the magic
        if (rcthr > 1200) // Throttle raised, turn on stablisation.
        {
            // Stablise PIDS
            Vector3f rateTargets = ac.Execute(attitudeTargets, attitude);

            // rate PIDS
            Vector3ui outputs = rrc.Execute(rateTargets, gyro);

            // mix pid outputs and send to the motors.
            long fl = rcthr + outputs.x + outputs.y + outputs.z;
            long bl = rcthr + outputs.x - outputs.y - outputs.z;
            long fr = rcthr - outputs.x + outputs.y - outputs.z;
            long br = rcthr - outputs.x - outputs.y + outputs.z;
            motors.SetFrontLeft(fl);
            motors.SetBackLeft(bl);
            motors.SetFrontRight(fr);
            motors.SetBackRight(br);

#ifdef TELEM
            b_led->write(0);
            long zero = 0;
            hal.console->write((uint8_t*)(&rcthr), 4);
            hal.console->write((uint8_t*)(&attitudeTargets.y), 4);
            hal.console->write((uint8_t*)(&attitudeTargets.x), 4);
            hal.console->write((uint8_t*)(&attitudeTargets.z), 4);

            hal.console->write((uint8_t*)(&gyro.y), 4);
            hal.console->write((uint8_t*)(&gyro.x), 4);
            hal.console->write((uint8_t*)(&gyro.z), 4);
            hal.console->write((uint8_t*)(&zero), 4);
            hal.console->write((uint8_t*)(&attitude.y), 4);
            hal.console->write((uint8_t*)(&attitude.x), 4);
            hal.console->write((uint8_t*)(&attitude.z), 4);

            hal.console->write((uint8_t*)(&rateTargets.y), 4);
            hal.console->write((uint8_t*)(&rateTargets.x), 4);
            hal.console->write((uint8_t*)(&rateTargets.z), 4);

            hal.console->write((uint8_t*)(&outputs.y), 4);
            hal.console->write((uint8_t*)(&outputs.x), 4);
            hal.console->write((uint8_t*)(&outputs.z), 4);

            hal.console->write((uint8_t*)(&fl), 4);
            hal.console->write((uint8_t*)(&fr), 4);
            hal.console->write((uint8_t*)(&bl), 4);
            hal.console->write((uint8_t*)(&br), 4);

            hal.console->write((uint8_t*)(rc.GetRaw()), 8);

            uint32_t ms = hal.scheduler->millis();
            uint32_t us = hal.scheduler->micros();

            hal.console->write((uint8_t*)(&ms), 4);
            hal.console->write((uint8_t*)(&us), 4);

            b_led->write(1);
#endif
        }
        else
        {
            motors.Stop();

            // reset PID integrals whilst on the ground
            ac.Reset();
            rrc.Reset();
        }
    }
    return 0;
}
