#pragma once
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>


inline std::string time_stamp() 
{
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

#ifdef _WIN32
    std::tm localTime;
    localtime_s(&localTime, &currentTime);
#else
    std::tm localTime;
    localtime_r(&currentTime, &localTime);
#endif

    std::ostringstream oss;
    oss << std::put_time(&localTime, "%Y-%m-%d__%H-%M-%S");
    return oss.str();
}