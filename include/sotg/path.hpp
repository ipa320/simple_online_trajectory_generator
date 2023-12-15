
#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/point.hpp"

namespace SOTG {

// A collection of points used as an input to the trajectory generator to
// quickly setup many new waypoints
class Path {
private:
    std::vector<Frame> waypoints_;

public:
    void addPoint(Frame point);
    void addPoint(const std::vector<Eigen::VectorXd>& old_point);
    void addPoint(const std::vector<std::vector<double>>& old_point);
    size_t getNumWaypoints() { return waypoints_.size(); };
    Frame& getPoint(size_t index);

    std::vector<Frame>::iterator begin() { return waypoints_.begin(); }
    std::vector<Frame>::iterator end() { return waypoints_.end(); }

    std::ostream& operator<<(std::ostream& out);
};

}  // namespace SOTG