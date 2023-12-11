#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace utility {
const double eps = 1e-6;

bool nearlyZero(double value);
bool nearlyEqual(double a, double b, double eps);
std::string vecToString(std::vector<double> vec);
double sign(double input);
}  // namespace utility