
#include "sotg/utility_functions.hpp"

bool utility::nearlyEqual(double a, double b, double eps) { return std::abs(b - a) < eps; }

bool utility::nearlyZero(double value) { return std::abs(value) < utility::eps; }

double utility::sign(double input)
{
    if (input > 0) {
        return 1;
    } else if (input < 0) {
        return -1;
    } else {
        return 0;
    }
}