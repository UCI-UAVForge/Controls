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

#include "RC.h"
#include "Util.h"

// Radio min/max values for each stick for my radio (worked out emperically)
const uint16_t THR_MIN = 1100;
const uint16_t THR_MAX = 2000;
const uint16_t YAW_MIN = 1100;
const uint16_t YAW_MAX = 1900;
const uint16_t PIT_MIN = 1100;
const uint16_t PIT_MAX = 1900;
const uint16_t ROL_MIN = 1100;
const uint16_t ROL_MAX = 1900;

namespace Quad
{
    RC::RC(const AP_HAL::HAL& hal)
        : hal(hal) {}

    void RC::Read()
    {
        hal.rcin->read(channels, 8);
    }

    long RC::GetThrottle()
    {
        return channels[2];
    }

    long RC::GetYaw()
    {
        return channels[3];
    }

    long RC::GetRoll()
    {
        return channels[0];
    }

    long RC::GetPitch()
    {
        return channels[1];
    }
    
    Vector3f RC::GetAttitudeInputs()
    {
        const float ROL_LIM = 45;
        const float PIT_LIM = 45;
        const float YAW_LIM = 180;

        return Vector3f(
            Util::Map((float)channels[0], ROL_MIN, ROL_MAX, -ROL_LIM, ROL_LIM),
            Util::Map((float)channels[1], PIT_MIN, PIT_MAX, -PIT_LIM, PIT_LIM),
            Util::Map((float)channels[3], YAW_MIN, YAW_MAX, -YAW_LIM, YAW_LIM));
    }

    uint16_t* RC::GetRaw()
    {
        return channels;
    }
}
