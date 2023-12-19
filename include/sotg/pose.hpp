
#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sotg/utility_functions.hpp"

namespace SOTG {

// Wraps std::vector and allows vector arithmetic, stores information about the unit of a particular value
class Pose {
private:
    Eigen::VectorXd location_;
    Eigen::Quaterniond orientation_;

public:
    Pose();
    Pose(const Eigen::VectorXd& loc, const Eigen::Quaterniond& rot);

    Pose diff(const Pose& f2) const;
    const Eigen::VectorXd& getLocation() const { return location_; }
    const Eigen::Quaterniond& getOrientation() const { return orientation_; }
    void setLocation(const Eigen::VectorXd& location) { location_ = location; }
    void setOrientation(const Eigen::Quaterniond& orientation) { orientation_ = orientation; }

    std::ostream& operator<<(std::ostream& out);
};

}  // namespace SOTG