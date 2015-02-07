// Copyright 2015 Jason Watkins & Gareth Owen
// This file is part of the UCI UAVForge Quad-copter Controls System. It is a
// modified version of the source code provided at
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
#include "RotationOrientationControl.h"

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

Quad::Motors motors = Quad::Motors(hal);
Quad::Instruments ins = Quad::Instruments(hal);
Quad::RC rc = Quad::RC(hal);

Quad::RotationRateControl rrc;
Quad::RotationOrientationControl roc;

void setup() {}

void loop()
{
    static float yaw_target = 0;
    ins.Update();

    // Read RC transmitter and map to sensible values
    rc.Read();
    long rcthr = rc.GetThrottle();
    long rcyaw = rc.GetYaw();
    long rcpit = rc.GetPitch();
    long rcroll = rc.GetRoll();

    // Ask MPU6050 for orientation
    Vector3f orientation = ins.GetOrientation();
    float roll = orientation.x;
    float pitch = orientation.y;
    float yaw = orientation.z;

    // Ask MPU6050 for gyro data
    Vector3f gyro = ins.GetGyro();
    float gyroPitch = gyro.y;
    float gyroRoll = gyro.x;
    float gyroYaw = gyro.z;

    // Do the magic
    if (rcthr > 10) // Throttle raised, turn on stablisation.
    {
        // Stablise PIDS
        roc.Execute((float)rcpit, (float)rcroll, (float)yaw_target, pitch, roll, yaw);
        float pitch_stab_output = roc.pitch;
        float roll_stab_output = roc.roll;
        float yaw_stab_output = roc.yaw;

        // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
        if (abs(rcyaw) > 5) {
            yaw_stab_output = rcyaw;
            yaw_target = yaw;   // remember this yaw for when pilot stops
        }

        // rate PIDS
        rrc.Execute(pitch_stab_output, roll_stab_output, yaw_stab_output,
            gyroPitch, gyroRoll, gyroYaw);
        long pitch_output = rrc.pitch;
        long roll_output = rrc.roll;
        long yaw_output = rrc.yaw;

        // mix pid outputs and send to the motors.
        motors.SetFrontLeft(rcthr + roll_output + pitch_output - yaw_output);
        motors.SetBackLeft(rcthr + roll_output - pitch_output + yaw_output);
        motors.SetFrontRight(rcthr - roll_output + pitch_output + yaw_output);
        motors.SetBackRight(rcthr - roll_output - pitch_output - yaw_output);
    }
    else
    {
        motors.Stop();

        // reset yaw target so we maintain this on takeoff
        yaw_target = yaw;

        // reset PID integrals whilst on the ground
        roc.Reset();
        rrc.Reset();
    }
}

AP_HAL_MAIN();
