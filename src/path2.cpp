
#include "sotg/path2.hpp"

#include <iostream>

using namespace SOTG;

void Path2::addPoint(Point2 point) { waypoints_.push_back(point); }

Point2& Path2::getPoint(std::string name)
{
    for (auto& point : waypoints_) {
        if (point.getName().compare(name) == 0) {
            return point;
        }
    }
    throw std::runtime_error("No Point named \"" + name + "\" was found");
}

std::ostream& Path2::operator<<(std::ostream& out)
{
    out << "[ ";

    if (!waypoints_.empty()) {
        for (size_t i = 0; i < waypoints_.size() - 1; i++) {
            out << waypoints_[i] << ", ";
        }
        out << waypoints_.back() << " ]";
    } else {
        out << " ]";
    }

    return out;
}