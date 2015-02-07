// Copyright 2015 Jason Watkins & Gareth Owen
// This file is part of the UCI UAVForge Quad-copter Controls System. It is
// based on the source code provided at 
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

#include "RC.h"
#include "Util.h"

// Radio min/max values for each stick for my radio (worked out emperically)
const uint16_t THR_MIN = 1070;
const uint16_t THR_MAX = 1915;
const uint16_t YAW_MIN = 1068;
const uint16_t YAW_MAX = 1915;
const uint16_t PIT_MIN = 1077;
const uint16_t PIT_MAX = 1915;
const uint16_t ROL_MIN = 1090;
const uint16_t ROL_MAX = 1913;

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
        return Util::ToPercentage(channels[2], THR_MIN, THR_MAX);
    }

    long RC::GetYaw()
    {
        return Util::Map(channels[3], YAW_MIN, YAW_MAX, -180, 180);
    }

    long RC::GetRoll()
    {
        return Util::Map(channels[0], ROL_MIN, ROL_MAX, -45, 45);
    }

    long RC::GetPitch()
    {
        return Util::Map(channels[1], PIT_MIN, PIT_MAX, -45, 45);
    }
}
