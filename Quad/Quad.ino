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
//#define RAW_THROTTLE
#define VELOCITY_CONTROL

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
#include "VelocityControl.h"
#include "AltitudeRateControl.h"
#include "AltitudeControl.h"

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

int main(void)
{
    hal.init(0, NULL);

    // Initialize LED's
    AP_HAL::DigitalSource* ledB;
    AP_HAL::DigitalSource* ledY;
    AP_HAL::DigitalSource* ledR;
    ledB = hal.gpio->channel(25);
    ledY = hal.gpio->channel(26);
    ledR = hal.gpio->channel(27);
    ledB->mode(GPIO_OUTPUT);
    ledY->mode(GPIO_OUTPUT);
    ledR->mode(GPIO_OUTPUT);

    // Yellow LED on indicates we are in startup
    ledB->write(1);
    ledY->write(0);
    ledR->write(1);

    Quad::Motors motors = Quad::Motors(hal);
    Quad::Instruments ins = Quad::Instruments(hal);
    Quad::RC rc = Quad::RC(hal);

    Quad::RotationRateControl rrc;
    Quad::AttitudeControl ac;
    Quad::VelocityControl vc;

    Quad::AltitudeRateControl altrc;
    Quad::AltitudeControl altc;
    

    hal.scheduler->system_initialized();

    // Red LED on indicates we are live
    ledY->write(1);
    ledR->write(0);

    float yawHoldAngle = 0;
    for (;;)
    {
        ins.Update();
        rc.Read();

        // Get sensor inputs
        Vector3f attitude = ins.GetAttitude();
        Vector3f gyro = ins.GetGyro();
        Vector3f position = ins.GetPosition();
        Vector3f velocity = ins.GetVelocity();
        Vector2f velocity2D = Vector2f(velocity.x, velocity.y);

        long throttle;
#ifdef RAW_THROTTLE
        // Use the raw RC input for throttle
        throttle = rc.GetThrottle();
#else
        // Calculate throttle based on desired altitude
        float targetAlt = rc.GetAltitudeInput();
        float targetAltRate = altc.Execute(targetAlt, position.z);
        throttle = altrc.Execute(targetAltRate, velocity.z);
#endif
        
        // Do the magic
        if (throttle > rc.GetThrottleMin() + 100) // Throttle raised, turn on stablisation.
        {
            Vector2f attitude2D;
#ifdef VELOCITY_CONTROL
            Vector2f rcVelocity = rc.GetVelocityInputs();
            attitude2D = vc.Execute(rcVelocity, velocity2D);
#else
            attitude2D = rc.GetAttitudeInputs();
#endif

            // Input targets
            Vector3f attitudeInputs = Vector3f(attitude2D.x, attitude2D.y, yawHoldAngle);

            // Stablise PIDS
            Vector3f rateTargets = ac.Execute(attitudeInputs, attitude);
            float rcYaw = rc.GetYawInput();
            if (abs(rcYaw) > 5) // Override yaw rate if there is a stick input.
            {
                rateTargets.z = rcYaw;
                yawHoldAngle = attitude.z;
            }

            // rate PIDS
            Vector3ui outputs = rrc.Execute(rateTargets, gyro);

            // mix pid outputs and send to the motors.
            long fl = throttle + outputs.x + outputs.y + outputs.z;
            long bl = throttle + outputs.x - outputs.y - outputs.z;
            long fr = throttle - outputs.x + outputs.y - outputs.z;
            long br = throttle - outputs.x - outputs.y + outputs.z;
            motors.SetFrontLeft(fl);
            motors.SetBackLeft(bl);
            motors.SetFrontRight(fr);
            motors.SetBackRight(br);

#ifdef TELEM
            ledB->write(0);
            long zero = 0;
            hal.console->write((uint8_t*)(&throttle), 4);
            hal.console->write((uint8_t*)(&attitude2D.y), 4);
            hal.console->write((uint8_t*)(&attitude2D.x), 4);
            hal.console->write((uint8_t*)(&rcYaw), 4);

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

            ledB->write(1);
#endif
        }
        else
        {
            motors.Stop();

            // reset PID integrals whilst on the ground
            vc.Reset();
            ac.Reset();
            rrc.Reset();

            altc.Reset();
            altrc.Reset();
        }
    }
    return 0;
}
