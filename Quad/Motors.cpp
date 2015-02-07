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

#include "Motors.h"

const int FL = 2;
const int FR = 0;
const int BL = 1;
const int BR = 3;

namespace Quad
{
    Motors::Motors(const AP_HAL::HAL& hal)
        : hal(hal)
    {
        // Enable the motors and set at 490Hz update
        hal.rcout->set_freq(0xF, 490);
        hal.rcout->enable_mask(0xFF);
    }

    void Motors::SetFrontLeft(long throttle)
    {
        hal.rcout->write(FL, throttle);
    }

    void Motors::SetFrontRight(long throttle)
    {
        hal.rcout->write(FR, throttle);
    }

    void Motors::SetBackLeft(long throttle)
    {
        hal.rcout->write(BL, throttle);
    }

    void Motors::SetBackRight(long throttle)
    {
        hal.rcout->write(BR, throttle);
    }

    void Motors::Stop()
    {
        SetFrontLeft(1000);
        SetFrontRight(1000);
        SetBackLeft(1000);
        SetBackRight(1000);
    }
}
