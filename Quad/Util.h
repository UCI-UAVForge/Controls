#pragma once


namespace Quad
{
    namespace Util
    {
        long Map(long x, long in_min, long in_max, long out_min, long out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        long ToPercentage(long x, long min, long max)
        {
            return (x - min) * 100 / (max - min);
        }
    }
}