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

#include <AP_Math.h>
#include <AP_HAL.h>

namespace Quad
{
    namespace Comms
    {
        // Defines return codes for the ReadPacket function.
        enum Validity
        {
            V_Valid,
            V_InvalidCheksum,
            V_InvalidType,
            V_UnexpectedLength,
            V_TooShort,
            V_NoData,
            V_InvalidInitialByte
        };

        // Defines valid packet type IDs. Note that since the packet type occupies only the low 4 bits of the first byte of a
        // packet, the maximum value for this enum should be no more than 15.
        enum PacketType
        {
            PT_Ping = 1,
            PT_Pong,
            PT_SS_Output,
            PT_SS_Input,
            PT_Scalar_16 = 8,
            PT_Scalar_32,
            PT_Vector2_16,
            PT_Vector2_32,
            PT_Vector3_16,
            PT_Vector3_32,
            PT_Vector4_16
        };

        class CommsHandler
        {
        public:
            CommsHandler(const AP_HAL::HAL& hal);

            Validity Read();

            void SendPing();

            void SendStartInputs(uint16_t flags);

            void SendScalar16(uint8_t type, uint16_t value);

            void SendScalar32(uint8_t type, uint32_t value);

            void SendScalarF(uint8_t type, float value);

            void SendVector2(uint8_t type, Vector2i value);

            void SendVector2(uint8_t type, Vector2ui value);

            void SendVector2(uint8_t type, Vector2<int32_t> value);

            void SendVector2(uint8_t type, Vector2<uint32_t> value);

            void SendVector2(uint8_t type, Vector2f value);

            void SendVector3(uint8_t type, Vector3i value);

            void SendVector3(uint8_t type, Vector3ui value);

            void SendVector3(uint8_t type, Vector3<int32_t> value);

            void SendVector3(uint8_t type, Vector3<uint32_t> value);

            void SendVector3(uint8_t type, Vector3f value);

            void SendVector4(uint8_t type, int16_t w, int16_t x, int16_t y, int16_t z);

            void SendVector4(uint8_t type, uint16_t w, uint16_t x, uint16_t y, uint16_t z);

            uint16_t GetOutputFlags();

        private:
            // Takes an action based on the type of packet.
            // packet: A pointer to a byte array containing a data packet.
            void HandlePacket();

            // Reads a packet from the Serial buffer and stores it at the specified address
            // packet: A pointer to a byte array of at least PACKET_MAX_LENGTH bytes.
            // Returns: A Validity value indicating success or a reason for failure.
            Validity ReadPacket(uint8_t* packet);

            uint8_t GetPayloadLength(PacketType id);

            bool ValidateChecksum(size_t len);
            void SetChecksum(uint8_t* buffer, size_t len);

            const AP_HAL::HAL& hal;
            uint8_t readBuffer[18];
            uint16_t outputFlags;
        };
    }
}