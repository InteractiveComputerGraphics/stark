#include "blends.h"

double stark::utils::blend(double min, double max, double duration, double begin_time, double current_time, BlendType blendType)
{
    const double dt = current_time - begin_time;

    // Error exit if time is out of range
    if (dt < 0.0 || dt > duration) {
        std::cout << "stark error: utils::blend() got time out of range: " << current_time << std::endl;
        exit(-1);
    }

    const double t = dt / duration;
    double result = min;

    switch (blendType) {
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
    case BlendType::Anticipation:
        // Anticipation with slight back movement before going forward
        result += (pow(t - 1, 3) + 1) * (max - min);
        break;
    case BlendType::Bounce:
        // Simple bounce effect, more sophisticated bounce can be added
        if (t < 0.5) {
            result += (1 - pow(1 - 2 * t, 2)) * (max - min) / 2;
        }
        else {
            result += (pow(2 * t - 1, 2) + 1) * (max - min) / 2;
        }
        break;
    default:
        std::cerr << "stark error: utils::blend() got unknown blend type." << std::endl;
        exit(1);
    }

    return result;
}
