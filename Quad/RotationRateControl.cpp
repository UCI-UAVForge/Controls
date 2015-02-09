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

#include "RotationRateControl.h"

namespace Quad
{
    RotationRateControl::RotationRateControl()
    {
        // PID Configuration
        pitchPID.kP(1);
        pitchPID.kI(0.01);
        pitchPID.imax(10);

        rollPID.kP(1);
        rollPID.kI(0.01);
        rollPID.imax(10);

        yawPID.kP(1.5);
        yawPID.kI(0.01);
        yawPID.imax(10);
    }

    void RotationRateControl::Execute(float tPitch, float tRoll, float tYaw, float gPitch, float gRoll, float gYaw)
    {
        pitch = (long)constrain(pitchPID.get_pid(tPitch - gPitch, 1), -500, 500);
        roll = (long)constrain(rollPID.get_pid(tRoll - gRoll, 1), -500, 500);
        yaw = (long)constrain(yawPID.get_pid(tYaw - gYaw, 1), -500, 500);
    }

    void RotationRateControl::Reset()
    {
        pitchPID.reset_I();
        rollPID.reset_I();
        yawPID.reset_I();
    }
}