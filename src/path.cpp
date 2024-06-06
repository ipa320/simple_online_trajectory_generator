#include "sotg/path.hpp"

#include <iostream>

using namespace SOTG;

void Path::addPoint(Point point) {
    int point_id = waypoints_.size() + 1;
    point.setID(point_id);
    waypoints_.push_back(point); }

// should be in pitasc glue
void Path::addPoint(const std::vector<Eigen::VectorXd>& old_point)
{
    Point new_point;
    // Expecting xyz, abc
    Eigen::VectorXd linear_point = old_point.front();
    Eigen::VectorXd angular_point = old_point.back();

    for (auto i = 0; i < linear_point.size(); ++i) {
        new_point.addValue(linear_point[i]);
    }
    new_point.setOrientationIndex(linear_point.size());
    for (auto i = 0; i < angular_point.size(); ++i) {
        new_point.addValue(angular_point[i]);
    }

    int point_id = waypoints_.size() + 1;
    new_point.setID(point_id);
    waypoints_.push_back(new_point);
}

void Path::addPoint(const std::vector<std::vector<double>>& old_point)
{
    Point new_point;

    std::vector<double> linear_point = old_point.front();
    std::vector<double> angular_point = old_point.back();

    for (size_t i = 0; i < linear_point.size(); ++i) {
        new_point.addValue(linear_point[i]);
    }
    new_point.setOrientationIndex(linear_point.size());
    for (size_t i = 0; i < angular_point.size(); ++i) {
        new_point.addValue(angular_point[i]);
    }

    int point_id = waypoints_.size() + 1;
    new_point.setID(point_id);
    waypoints_.push_back(new_point);
}

Point& Path::getPointReference(size_t index)
{
    if (index > waypoints_.size() - 1) {
        throw std::runtime_error("Index out of bounds, trying to access " + std::to_string(index)
                                 + "# waypoint from a path with " + std::to_string(waypoints_.size())
                                 + " waypoints total");
    }
    return waypoints_[index];
}


Point SOTG::Path::getPointValue(size_t index) const
{
    return waypoints_[index];
}

Path SOTG::Path::operator+(const Path& path)
{
    for(size_t i = 0; i < path.size(); i++)
    {
        addPoint(path.getPointValue(i));
    }

    return *this;
}

std::ostream& Path::operator<<(std::ostream& out)
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