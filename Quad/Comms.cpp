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

#include "Comms.h"

namespace Quad
{
    namespace Comms
    {
        CommsHandler::CommsHandler(const AP_HAL::HAL& hal)
            : hal(hal) {}

        Validity CommsHandler::Read()
        {
            int initial = hal.console->read();
            if (initial == -1)
            {
                return V_NoData;
            }
            if (initial == 0 || initial == 255)
            {
                return V_InvalidInitialByte;
            }

            readBuffer[0] = (uint8_t)initial;
            PacketType id = (PacketType)(initial & 0x0F);
            uint8_t packetLength = initial >> 4;
            uint8_t expectedLength = GetPayloadLength(id);
            size_t bytesRead = 1;
            int nextByte = hal.console->read();
            for (int i = 1; nextByte != -1 && i < 18; ++i)
            {
                readBuffer[i] = (uint8_t)nextByte;
                ++bytesRead;
                if (bytesRead == packetLength + 3)
                {
                    break;
                }
                nextByte = hal.console->read();
            }
            if (expectedLength == -1)
            {
                return V_InvalidType;
            }
            if (expectedLength != packetLength)
            {
                return V_UnexpectedLength;
            }
            if (bytesRead < (packetLength + 3))
            {
                return V_TooShort;
            }
            if (!ValidateChecksum(packetLength + 3))
            {
                return V_InvalidCheksum;
            }

            HandlePacket();
            return V_Valid;
        }

        void CommsHandler::HandlePacket()
        {
            PacketType pt = (PacketType)(readBuffer[0] & 0x0F);
            switch (pt)
            {
            case PT_Ping:
                break;
            case PT_Pong:
                break;
            case PT_SS_Output:
                outputFlags = *((uint16_t*)(readBuffer + 1));
                break;
            case PT_SS_Input:
                break;
            case PT_Scalar_16:
                break;
            case PT_Scalar_32:
                break;
            case PT_Vector2_16:
                break;
            case PT_Vector2_32:
                break;
            case PT_Vector3_16:
                break;
            case PT_Vector3_32:
                break;
            case PT_Vector4_16:
                break;
            default:
                break;
            }
        }

        void CommsHandler::SendPing()
        {
            uint8_t buffer[3];
            buffer[0] = (uint8_t)PT_Ping;
            SetChecksum(buffer, 1);
            hal.console->write(buffer, 3);

        }

        void CommsHandler::SendStartInputs(uint16_t flags)
        {
            uint8_t buffer[5];
            buffer[0] = (uint8_t)PT_SS_Input | 0x20;
            *((uint16_t*)(buffer + 1)) = flags;
            SetChecksum(buffer, 3);
            hal.console->write(buffer, 5);
            
        }

        void CommsHandler::SendScalar16(uint8_t type, uint16_t value)
        {
            uint8_t buffer[6];
            buffer[0] = (uint8_t)PT_Scalar_16 | 0x30;
            *((uint8_t*)(buffer + 1)) = type;
            *((uint16_t*)(buffer + 2)) = value;
            SetChecksum(buffer, 4);
            hal.console->write(buffer, 6);            
        }

        void CommsHandler::SendScalar32(uint8_t type, uint32_t value)
        {
            uint8_t buffer[8];
            buffer[0] = (uint8_t)PT_Scalar_32 | 0x50;
            buffer[1] = type;
            *((uint32_t*)(buffer + 2)) = value;
            SetChecksum(buffer, 6);
            hal.console->write(buffer, 8);
        }

        void CommsHandler::SendScalarF(uint8_t type, float value)
        {
            SendScalar32(type, *((uint32_t*)&value));
        }

        void CommsHandler::SendVector2(uint8_t type, Vector2i value)
        {
            uint8_t buffer[8];
            buffer[0] = (uint8_t)PT_Vector2_16 | 0x50;
            *((uint8_t*)(buffer + 1)) = type;
            *((Vector2i*)(buffer + 2)) = value;
            SetChecksum(buffer, 8);
            hal.console->write(buffer, 8);
        }

        void CommsHandler::SendVector2(uint8_t type, Vector2ui value)
        {
            SendVector2(type, *((Vector2i*)&value));
        }

        void CommsHandler::SendVector2(uint8_t type, Vector2<int32_t> value)
        {
            uint8_t buffer[12];
            buffer[0] = (uint8_t)PT_Vector2_32 | 0x90;
            *((uint8_t*)(buffer + 1)) = type;
            *((Vector2<int32_t>*)(buffer + 2)) = value;
            SetChecksum(buffer, 10);
            hal.console->write(buffer, 12);            
        }

        void CommsHandler::SendVector2(uint8_t type, Vector2<uint32_t> value)
        {
            SendVector2(type, *((Vector2<int32_t>*)&value));
        }

        void CommsHandler::SendVector2(uint8_t type, Vector2f value)
        {
            SendVector2(type, *((Vector2<int32_t>*)&value));
        }

        void CommsHandler::SendVector3(uint8_t type, Vector3i value)
        {
            uint8_t buffer[10];
            buffer[0] = (uint8_t)PT_Vector3_16 | 0x70;
            *((uint8_t*)(buffer + 1)) = type;
            *((Vector3i*)(buffer + 2)) = value;
            SetChecksum(buffer, 8);
            hal.console->write(buffer, 10);            
        }

        void CommsHandler::SendVector3(uint8_t type, Vector3ui value)
        {
            SendVector3(type, *((Vector3i*)&value));
        }

        void CommsHandler::SendVector3(uint8_t type, Vector3<int32_t> value)
        {
            uint8_t buffer[16];
            buffer[0] = (uint8_t)PT_Vector3_32 | 0xD0;
            *((uint8_t*)(buffer + 1)) = type;
            *((Vector3<int32_t>*)(buffer + 2)) = value;
            SetChecksum(buffer, 14);
            hal.console->write(buffer, 16);
        }

        void CommsHandler::SendVector3(uint8_t type, Vector3<uint32_t> value)
        {
            SendVector3(type, *((Vector3<int32_t>*)&value));
        }

        void CommsHandler::SendVector3(uint8_t type, Vector3f value)
        {
            SendVector3(type, *((Vector3<int32_t>*)&value));
        }

        void CommsHandler::SendVector4(uint8_t type, int16_t w, int16_t x, int16_t y, int16_t z)
        {
            uint8_t buffer[12];
            buffer[0] = (uint8_t)PT_Vector4_16 | 0x90;
            *((uint8_t*)(buffer + 1)) = type;
            *((int16_t*)(buffer + 2)) = w;
            *((int16_t*)(buffer + 4)) = x;
            *((int16_t*)(buffer + 6)) = y;
            *((int16_t*)(buffer + 8)) = z;
            SetChecksum(buffer, 10);
            hal.console->write(buffer, 12);
        }

        void CommsHandler::SendVector4(uint8_t type, uint16_t w, uint16_t x, uint16_t y, uint16_t z)
        {
            uint8_t buffer[12];
            buffer[0] = (uint8_t)PT_Vector4_16 | 0x90;
            buffer[1] = type;
            *((uint16_t*)(buffer + 2)) = w;
            *((uint16_t*)(buffer + 4)) = x;
            *((uint16_t*)(buffer + 6)) = y;
            *((uint16_t*)(buffer + 8)) = z;
            SetChecksum(buffer, 10);
            hal.console->write(buffer, 12);
        }

        uint16_t CommsHandler::GetOutputFlags()
        {
            return outputFlags;
        }

        // Maps a packet type identifier to a packet length
        // Returns: The total number of bytes in a packet.
        uint8_t CommsHandler::GetPayloadLength(PacketType id)
        {
            switch (id)
            {
            case PT_Ping:
            case PT_Pong:
                return 0;
            case PT_SS_Output:
            case PT_SS_Input:
                return 2;
            case PT_Scalar_16:
                return 3;
            case PT_Scalar_32:
            case PT_Vector2_16:
                return 5;
            case PT_Vector2_32:
            case PT_Vector4_16:
                return 9;
            case PT_Vector3_16:
                return 7;
            case PT_Vector3_32:
                return 13;
            default:
                return -1;
            }
        }

        // Appends a Fletcher-16 checksum to the specified packet. See http://en.wikipedia.org/wiki/Fletcher%27s_checksum
        // for more information. The checksum is appended in check-byte format. i.e. recalculating the checksum over the
        // entire packet will yield 0 for both sums.
        void CommsHandler::SetChecksum(uint8_t* data, size_t bytes)
        {
            uint16_t sum1 = 0;
            uint16_t sum2 = 0;
            uint8_t* ptr = data;
            size_t len = bytes;
            while (len--)
            {
                sum1 = (sum1 + *ptr++) % 255;
                sum2 = (sum2 + sum1) % 255;
            }
            uint8_t check1 = 255 - ((sum1 + sum2) % 255);
            uint8_t check2 = 255 - ((sum1 + check1) % 255);
            data[bytes] = check1;
            data[bytes + 1] = check2;
        }

        // Checks that the Fletcher-16 checksum of a packet is 0.
        bool CommsHandler::ValidateChecksum(size_t bytes)
        {
            uint8_t* data = readBuffer;
            uint16_t sum1 = 0;
            uint16_t sum2 = 0;
            while (bytes--)
            {
                sum1 = (sum1 + *data++) % 255;
                sum2 = (sum2 + sum1) % 255;
            }
            return sum1 == 0 && sum2 == 0;
        }
    }
}