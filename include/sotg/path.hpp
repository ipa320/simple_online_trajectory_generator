
#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/pose.hpp"

namespace SOTG {

// A collection of points used as an input to the trajectory generator to
// quickly setup many new waypoints
class Path {
private:
    std::vector<Pose> waypoints_;

public:
    void addPoint(const Pose& pose);
    void addPoint(const Eigen::VectorXd& loc, const Eigen::Quaterniond& rot);
    size_t getNumWaypoints() { return waypoints_.size(); };
    Pose& getPoint(size_t index);

    std::vector<Pose>::iterator begin() { return waypoints_.begin(); }
    std::vector<Pose>::iterator end() { return waypoints_.end(); }

    std::ostream& operator<<(std::ostream& out);
};

}  // namespace SOTG