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
