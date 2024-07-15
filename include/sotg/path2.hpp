#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/point2.hpp"

namespace SOTG {

// A collection of points used as an input to the trajectory generator to
// quickly setup many new waypoints
class Path2 {
private:
    std::vector<Point2> waypoints_;

public:
    void addPoint(Point2 point);

    size_t getNumWaypoints() { return waypoints_.size(); };
    Point2& getPoint(std::string name);
    Point2& getPoint(size_t index) { return waypoints_[index]; }

    std::vector<Point2>::iterator begin() { return waypoints_.begin(); }
    std::vector<Point2>::iterator end() { return waypoints_.end(); }

    std::ostream& operator<<(std::ostream& out);
};

}  // namespace SOTG