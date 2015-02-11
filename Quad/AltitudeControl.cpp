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

#include "AltitudeControl.h"

namespace Quad
{
    AltitudeControl::AltitudeControl()
    {
        altPID.kP(10);
    }

    float AltitudeControl::Execute(float target, float actual)
    {
        float error = target - actual;
        return constrain(altPID.get_pid(error, 1), -10, 10);
    }

    void AltitudeControl::Reset()
    {
        altPID.reset_I();
    }
}