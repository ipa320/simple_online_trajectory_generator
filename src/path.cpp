#include "sotg/path.hpp"

#include <iostream>

#include "sotg/pose.hpp"

using namespace SOTG;

void Path::addPoint(const Eigen::VectorXd& loc, const Eigen::Quaterniond& rot)
{
    Pose new_pose(loc, rot);

    waypoints_.push_back(new_pose);
}

void Path::addPoint(const Pose& pose) { waypoints_.push_back(pose); }

Pose& Path::getPoint(size_t index)
{
    if (index > waypoints_.size() - 1) {
        throw std::runtime_error("Index out of bounds, trying to access " + std::to_string(index)
                                 + "# waypoint from a path with " + std::to_string(waypoints_.size())
                                 + " waypoints total");
    }
    return waypoints_[index];
}

// std::ostream& Path::operator<<(std::ostream& out)
// {
//     out << "[ ";

//     if (!waypoints_.empty()) {
//         for (size_t i = 0; i < waypoints_.size() - 1; i++) {
//             out << waypoints_[i] << ", ";
//         }
//         out << waypoints_.back() << " ]";
//     } else {
//         out << " ]";
//     }

//     return out;
// }