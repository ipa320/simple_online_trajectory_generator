#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace utility {
const double eps = 1e-6;

bool nearlyZero(double value);
bool nearlyEqual(double a, double b, double eps);
double sign(double input);
Eigen::Quaterniond quatDiff(const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2);
}  // namespace utility