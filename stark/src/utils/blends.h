#pragma once
#include <iostream>

namespace stark
{
    enum class BlendType 
    {
        Instant,
        Linear,
        EaseIn,
        EaseOut,
        EaseInOut
    };

    double blend(double min, double max, double begin_time, double end_time, double current_time, BlendType blendType);
}
