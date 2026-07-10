#pragma once
#include <chrono>
#include <string>
#include <iostream>


class Timer {
public:
    Timer(std::string name) : _name(name) { _start = std::chrono::system_clock::now(); }

    ~Timer() {
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - _start);
    
        double duration_in_seconds =
            double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
        std::cout << "[TickTime]Step:" << _name <<"cost:"<< duration_in_seconds << " s" << std::endl;
    }

private:
    std::string _name;
    std::chrono::time_point<std::chrono::system_clock> _start;
};
