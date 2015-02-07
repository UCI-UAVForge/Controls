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

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

Quad::Motors motors = Quad::Motors(hal);
Quad::Instruments ins = Quad::Instruments(hal);
Quad::RC rc = Quad::RC(hal);

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

void setup()
{
    // PID Configuration
    pids[PID_PITCH_RATE].kP(0.7);
    pids[PID_PITCH_RATE].kI(1);
    pids[PID_PITCH_RATE].imax(50);

    pids[PID_ROLL_RATE].kP(0.7);
    pids[PID_ROLL_RATE].kI(1);
    pids[PID_ROLL_RATE].imax(50);

    pids[PID_YAW_RATE].kP(2.7);
    pids[PID_YAW_RATE].kI(1);
    pids[PID_YAW_RATE].imax(50);

    pids[PID_PITCH_STAB].kP(4.5);
    pids[PID_ROLL_STAB].kP(4.5);
    pids[PID_YAW_STAB].kP(10);

    // We're ready to go! Now over to loop()
}

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
    float gyroPitch = ToDeg(gyro.y);
    float gyroRoll = ToDeg(gyro.x);
    float gyroYaw = ToDeg(gyro.z);

    // Do the magic
    if (rcthr > 10) {  // Throttle raised, turn on stablisation.
        // Stablise PIDS
        float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250);
        float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
        float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

        // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
        if (abs(rcyaw) > 5) {
            yaw_stab_output = rcyaw;
            yaw_target = yaw;   // remember this yaw for when pilot stops
        }

        // rate PIDS
        long pitch_output = (long)constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);
        long roll_output = (long)constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);
        long yaw_output = (long)constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);

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
        for (int i = 0; i < 6; i++)
        {
            pids[i].reset_I();
        }
    }
}

AP_HAL_MAIN();
