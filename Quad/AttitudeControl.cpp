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

#include "AttitudeControl.h"

namespace Quad
{
    float Wrap180(float value)
    {
        if (value < -180)
        {
            return value + 360;
        }
        if (value > 180)
        {
            return value - 360;
        }
        return value;
    }

    AttitudeControl::AttitudeControl()
    {
        // PID Configuration
        pitchPID.kP(4.5);

        rollPID.kP(4.5);
    }

    void AttitudeControl::Execute(float tPitch, float tRoll, float oPitch, float oRoll)
    {
        pitch = constrain(pitchPID.get_pid(tPitch - oPitch, 1), -250, 250);
        roll = constrain(rollPID.get_pid(tRoll - oRoll, 1), -250, 250);
    }

    void AttitudeControl::Reset()
    {
        pitchPID.reset_I();
        rollPID.reset_I();
    }
}