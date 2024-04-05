#include "blends.h"

#include <cmath>

double stark::blend(double min, double max, double begin_time, double end_time, double current_time, BlendType blendType)
{
    const double duration = end_time - begin_time;
    const double dt = current_time - begin_time;

    // Error exit if time is out of range
    if (dt < 0.0 || dt > duration) {
        std::cout << "stark error: blend() got time out of range: " << current_time << std::endl;
        exit(-1);
    }

    const double t = dt / duration;
    double result = min;

    switch (blendType) {
    case BlendType::Instant:
        result = max;
        break;
    case BlendType::Linear:
        result += t * (max - min);
        break;
    case BlendType::EaseIn:
        result += pow(t, 3) * (max - min);
        break;
    case BlendType::EaseOut:
        result += (1 - pow(1 - t, 3)) * (max - min);
        break;
    case BlendType::EaseInOut:
        if (t < 0.5) {
            result += 4 * pow(t, 3) * (max - min);
        }
        else {
            result += (1 - pow(-2 * t + 2, 3) / 2) * (max - min);
        }
        break;
    default:
        std::cerr << "stark error: blend() got unknown blend type." << std::endl;
        exit(1);
    }

    return result;
}
