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
    struct ControlData
    {
        uint16_t MotorFrontLeft;
        uint16_t MotorFrontRight;
        uint16_t MotorBackLeft;
        uint16_t MotorBackRight;

        float PitchRate;
        float RollRate;
        float YawRate;

        float PitchAngle;
        float RollAngle;
        float YawAngle;

        float XVelocity;
        float YVelocity;
        float ZVelocity;
    };
}