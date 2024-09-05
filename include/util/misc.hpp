// misc.hpp
// Contains miscellaneous utility methods

#include <chrono>

#ifndef MISC_HPP
#define MISC_HPP


inline double scale(double n, double newMin, double newMax, double oldMin, double oldMax) {
    return ((newMax - newMin) * (n - oldMin) / (oldMax - oldMin)) + newMin;
}

inline void sleep(int micros) {
    std::this_thread::sleep_for(std::chrono::microseconds(micros));
}


#endif