#pragma once
#include <iostream>

namespace stark::utils
{
    enum class BlendType 
    {
        Instant,
        Linear,
        EaseIn,
        EaseOut,
        EaseInOut,
        Anticipation,
        Bounce
    };

    double blend(double min, double max, double duration, double begin_time, double current_time, BlendType blendType);
}
