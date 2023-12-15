
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
class Frame {
private:
    Eigen::VectorXd location_;
    Eigen::Quaterniond orientation_;

public:
    Frame();
    Frame(std::vector<Eigen::VectorXd> vec_list);
    Frame(Eigen::VectorXd& vec);
    Frame(Eigen::VectorXd lin, Eigen::Quaterniond ang);

    Frame diff(const Frame& f2) const;
    const Eigen::VectorXd& getLocation() const { return location_; }
    const Eigen::Quaterniond& getOrientation() const { return orientation_; }
    void setLocation(const Eigen::VectorXd& location) { location_ = location; }
    void setOrientation(const Eigen::VectorXd& orientation) { orientation_ = orientation; }
    double getAngularDistance() const { }
};

}  // namespace SOTG