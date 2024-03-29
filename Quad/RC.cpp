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
const uint16_t GAIN_MIN = 968;
const uint16_t GAIN_MAX = 2074;

namespace Quad
{
    RC::RC(const AP_HAL::HAL& hal)
        : hal(hal) {}

    void RC::Read()
    {
        hal.rcin->read(channels, 8);
    }

    uint16_t RC::GetThrottle()
    {
        return channels[2];
    }

    uint16_t RC::GetYaw()
    {
        return channels[3];
    }

    uint16_t RC::GetRoll()
    {
        return channels[0];
    }

    uint16_t RC::GetPitch()
    {
        return channels[1];
    }

    uint16_t RC::GetGain()
    {
        return channels[5];
    }

    float RC::GetGainMultiplier()
    {
        return Util::Map((float)channels[5], GAIN_MIN, GAIN_MAX, 0, 2);
    }
    
    float RC::GetAltitudeInput()
    {
        const float ALT_LIM = 5;
        
        return Util::Map((float)channels[2], THR_MIN, THR_MAX, 0, ALT_LIM);
    }

    Vector2f RC::GetAttitudeInputs()
    {
        const float ROL_LIM = 45;
        const float PIT_LIM = 45;

        return Vector2f(
            Util::Map((float)channels[0], ROL_MIN, ROL_MAX, -ROL_LIM, ROL_LIM),
            Util::Map((float)channels[1], PIT_MIN, PIT_MAX, -PIT_LIM, PIT_LIM));
    }

    float RC::GetYawInput()
    {
        const float YAW_LIM = 180;

        return Util::Map((float)channels[3], YAW_MIN, YAW_MAX, -YAW_LIM, YAW_LIM);
    }
    
    Vector2f RC::GetVelocityInputs()
    {
        const float X_LIM = 2;
        const float Y_LIM = 2;

        return Vector2f(
            Util::Map((float)channels[0], ROL_MIN, ROL_MAX, -X_LIM, X_LIM),
            Util::Map((float)channels[1], PIT_MIN, PIT_MAX, -Y_LIM, Y_LIM));
    }

    uint16_t* RC::GetRaw()
    {
        return channels;
    }

    uint16_t RC::GetThrottleMin()
    {
        return THR_MIN;
    }
}
