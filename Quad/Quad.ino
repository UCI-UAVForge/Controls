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


//#define TELEM
#define RAW_THROTTLE
//#define VELOCITY_CONTROL

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

#include "Comms.h"
#include "Motors.h"
#include "Instruments.h"
#include "RC.h"
#include "Util.h"

#include "RotationRateControl.h"
#include "AttitudeControl.h"
#include "VelocityControl.h"
#include "AltitudeRateControl.h"
#include "AltitudeControl.h"

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

Quad::Comms::CommsHandler comms = Quad::Comms::CommsHandler(hal);

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

    Quad::RotationRateControl rrc = Quad::RotationRateControl(rc);
    Quad::AttitudeControl ac;
    Quad::VelocityControl vc;

    Quad::AltitudeRateControl altrc;
    Quad::AltitudeControl altc;
    

    hal.scheduler->system_initialized();

    // Red LED on indicates we are live
    ledY->write(1);
    ledR->write(0);

    uint8_t iterCounter;
    float yawHoldAngle = 0;
    for (;;)
    {
        ++iterCounter;
        iterCounter %= 50;
        Quad::Comms::Validity result = comms.Read();
        uint16_t flags = comms.GetOutputFlags();
        ledB->write(flags ? 0 : 1);

        ins.Update();
        rc.Read();

        // Get sensor inputs
        Vector3f attitude = ins.GetAttitude();
        Vector3f gyro = ins.GetGyro();
        Vector3f position = ins.GetPosition();
        Vector3f velocity = ins.GetVelocity();
        Vector2f velocity2D = Vector2f(velocity.x, velocity.y);

        uint16_t throttle;
#ifdef RAW_THROTTLE
        // Use the raw RC input for throttle
        throttle = rc.GetThrottle();
#else
        // Calculate throttle based on desired altitude
        float targetAlt = rc.GetAltitudeInput();
        float targetAltRate = altc.Execute(targetAlt, position.z);
        throttle = altrc.Execute(targetAltRate, velocity.z);
#endif
        
        Vector3i outputs;
        Vector3f rateTargets;
        uint16_t fl = 0;
        uint16_t bl = 0;
        uint16_t fr = 0;
        uint16_t br = 0;
        uint32_t lastTime = hal.scheduler->micros();
        uint32_t time;
        // Do the magic
        if (throttle > rc.GetThrottleMin() + 100) // Throttle raised, turn on stablisation.
        {
            time = hal.scheduler->micros();

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
            rateTargets = ac.Execute(attitudeInputs, attitude);
            float rcYaw = rc.GetYawInput();
            //if (abs(rcYaw) > 5) // Override yaw rate if there is a stick input.
            //{
            //    rateTargets.z = rcYaw;
            //    yawHoldAngle = attitude.z;
            //}
            rateTargets.z = rcYaw;

            // rate PIDS
            outputs = rrc.Execute(rateTargets, gyro);

            // mix pid outputs and send to the motors.
            float dt = (time - lastTime) / 1000000;
            uint16_t newFL = min(1200, throttle + outputs.x + outputs.y + outputs.z);
            uint16_t newBL = min(1200, throttle + outputs.x - outputs.y - outputs.z);
            uint16_t newFR = min(1200, throttle - outputs.x + outputs.y - outputs.z);
            uint16_t newBR = min(1200, throttle - outputs.x - outputs.y + outputs.z);

            Quad::Util::Filter(newFL, fl, dt);
            Quad::Util::Filter(newBL, bl, dt);
            Quad::Util::Filter(newFR, fr, dt);
            Quad::Util::Filter(newBR, br, dt);

            motors.SetFrontLeft(fl);
            motors.SetBackLeft(bl);
            motors.SetFrontRight(fr);
            motors.SetBackRight(br);
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
        lastTime = time;
        
        switch (iterCounter)
        {
        case 0:
            if (flags & 0x0001)
            {
                comms.SendScalar32(0x01, hal.scheduler->micros());
                break;
            }
        case 1:
            if (flags & 0x0002)
            {
                comms.SendVector4(0x01, rc.GetThrottle(), rc.GetRoll(), rc.GetPitch(), rc.GetYaw());
                break;
            }
        case 3:
            if (flags & 0x0008)
            {
                comms.SendVector4(0x03, fl, bl, fr, br);
                break;
            }
        case 5:
            if (flags & 0x0010)
            {
                comms.SendVector3(0x01, gyro);
                break;
            }
        case 6:
            if (flags & 0x0020)
            {
                comms.SendVector3(0x02, attitude);
                break;
            }
        case 7:
            if (flags & 0x0040)
            {
                float heading = ins.GetHeading();
                comms.SendScalarF(0x02, heading);
                break;
            }
        case 10:
            if (flags & 0x0200)
            {
                comms.SendVector3(0x05, position);
                break;
            }
        case 12:
            if (flags & 0x0800)
            {
                comms.SendVector3(0x01, outputs);
                break;
            }
        case 13:
            if (flags & 0x1000)
            {
                comms.SendVector3(0x08, rateTargets);
                break;
            }
        case 20:
            comms.SendScalar16(0x01, rc.GetGain());
            break;
        default:
            break;
        }
    }
    return 0;
}
