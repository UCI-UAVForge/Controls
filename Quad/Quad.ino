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
#include <AP_InertialSensor.h>

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
AP_HAL::DigitalSource* b_led;
AP_HAL::DigitalSource* y_led;
AP_HAL::DigitalSource* r_led;

int main(void)
{
    hal.init(0, NULL);


    b_led = hal.gpio->channel(25);
    y_led = hal.gpio->channel(26);
    r_led = hal.gpio->channel(27);
    b_led->mode(GPIO_OUTPUT);
    y_led->mode(GPIO_OUTPUT);
    r_led->mode(GPIO_OUTPUT);

    b_led->write(1);
    y_led->write(0);
    r_led->write(1);

    Quad::Motors motors = Quad::Motors(hal);
    Quad::Instruments ins = Quad::Instruments(hal);
    Quad::RC rc = Quad::RC(hal);

    Quad::RotationRateControl rrc;
    Quad::AttitudeControl ac;

    y_led->write(1);
    r_led->write(0);
    

    hal.scheduler->system_initialized();
    for (;;)
    {
        ins.Update();

        // Read RC transmitter and map to sensible values
        rc.Read();
        long rcthr = rc.GetThrottle();
        float rcyaw = rc.GetYaw();
        float rcpit = rc.GetPitch();
        float rcroll = rc.GetRoll();

        // Ask MPU6050 for orientation
        Vector3f orientation = ins.GetOrientation();
        float roll = orientation.x;
        float pitch = orientation.y;

        // Ask MPU6050 for gyro data
        Vector3f gyro = ins.GetGyro();
        float gyroRoll = gyro.x;
        float gyroPitch = gyro.y;
        float gyroYaw = gyro.z;

        // Do the magic
        if (rcthr > 1200) // Throttle raised, turn on stablisation.
        {
            // Stablise PIDS
            ac.Execute((float)rcpit, (float)rcroll, pitch, roll);
            float pitch_stab_output = ac.pitch;
            float roll_stab_output = ac.roll;
            float yaw_stab_output = abs(rcyaw) > 5 ? rcyaw : 0;

            // rate PIDS
            rrc.Execute(pitch_stab_output, roll_stab_output, yaw_stab_output,
                gyroPitch, gyroRoll, gyroYaw);
            long pitch_output = rrc.pitch;
            long roll_output = rrc.roll;
            long yaw_output = abs(rrc.yaw) > 10 ? rrc.yaw : 0;

            // mix pid outputs and send to the motors.
            long fl = rcthr + roll_output + pitch_output + yaw_output;
            long bl = rcthr + roll_output - pitch_output - yaw_output;
            long fr = rcthr - roll_output + pitch_output - yaw_output;
            long br = rcthr - roll_output - pitch_output + yaw_output;
            motors.SetFrontLeft(fl);
            motors.SetBackLeft(bl);
            motors.SetFrontRight(fr);
            motors.SetBackRight(br);

#ifdef TELEM
            b_led->write(0);
            long zero = 0;
            hal.console->write((uint8_t*)(&rcthr), 4);
            hal.console->write((uint8_t*)(&rcpit), 4);
            hal.console->write((uint8_t*)(&rcroll), 4);
            hal.console->write((uint8_t*)(&rcyaw), 4);

            hal.console->write((uint8_t*)(&gyroPitch), 4);
            hal.console->write((uint8_t*)(&gyroRoll), 4);
            hal.console->write((uint8_t*)(&gyroYaw), 4);
            hal.console->write((uint8_t*)(&zero), 4);
            hal.console->write((uint8_t*)(&pitch), 4);
            hal.console->write((uint8_t*)(&roll), 4);
            hal.console->write((uint8_t*)(&zero), 4);

            hal.console->write((uint8_t*)(&pitch_stab_output), 4);
            hal.console->write((uint8_t*)(&roll_stab_output), 4);
            hal.console->write((uint8_t*)(&yaw_stab_output), 4);

            hal.console->write((uint8_t*)(&pitch_output), 4);
            hal.console->write((uint8_t*)(&roll_output), 4);
            hal.console->write((uint8_t*)(&yaw_output), 4);

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
