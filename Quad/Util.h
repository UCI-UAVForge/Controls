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

#pragma once

namespace Quad
{
    namespace Util
    {
        long Map(long x, long in_min, long in_max, long out_min, long out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        float Map(float x, float in_min, float in_max, float out_min, float out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        long ToPercentage(long x, long min, long max)
        {
            return (x - min) * 100 / (max - min);
        }

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

        // f_cut = 10 Hz -> _filter = 15.9155e-3
        // f_cut = 15 Hz -> _filter = 10.6103e-3
        // f_cut = 20 Hz -> _filter =  7.9577e-3
        // f_cut = 25 Hz -> _filter =  6.3662e-3
        // f_cut = 30 Hz -> _filter =  5.3052e-3
        const float rc = 5.3052e-3; // Set to  "1 / ( 2 * PI * f_cut )";
        void Filter(uint16_t current, uint16_t& last, float dt)
        {
            last = last + (dt / (rc + dt)) * (current - last);
        }
    }
}