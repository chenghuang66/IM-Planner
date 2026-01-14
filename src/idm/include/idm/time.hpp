#pragma once

#include <chrono>

// ###################################################
// timer
// ###################################################

enum TimerUint
{
    SEC,
    MILLISEC,
    MICROSEC,
    NANOSEC
};

class Timer
{
public:
    void begin()
    {
        start = std::chrono::system_clock::now();
    }
    double end(const TimerUint &unit)
    {
        final = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(final - start);
        double dur = -1.0F;
        switch (unit)
        {
        case SEC:
            dur = (double)duration.count() * std::chrono::nanoseconds::period::num / std::chrono::nanoseconds::period::den;
            break;
        case MILLISEC:
            dur = (double)duration.count() * std::chrono::nanoseconds::period::num / std::chrono::microseconds::period::den;
            break;
        case MICROSEC:
            dur = (double)duration.count() * std::chrono::nanoseconds::period::num / std::chrono::milliseconds::period::den;
            break;
        case NANOSEC:
            dur = (double)duration.count() * std::chrono::nanoseconds::period::num;
        default:
            break;
        }
        return dur;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> final;
};