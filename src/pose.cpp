#include "sotg/pose.hpp"

using namespace SOTG;

Pose::Pose() { }

Pose::Pose(const Eigen::VectorXd& loc, const Eigen::Quaterniond& rot)
    : location_(loc)
    , orientation_(rot)
{
}

Pose Pose::diff(const Pose& f2) const
{
    Eigen::VectorXd lin = f2.getLocation() - location_;
    Eigen::Quaterniond ang = utility::quatDiff(orientation_, f2.getOrientation());
    Pose new_pose(lin, ang);
    return new_pose;
}

std::ostream& Pose::operator<<(std::ostream& out)
{
    out << "[ ";

    for (Eigen::Index i = 0; i < location_.size() - 1; ++i) {
        out << location_[i] << ", ";
    }
    out << location_[location_.size() - 1] << " | ";

    out << orientation_.w() << ", " << orientation_.x() << ", " << orientation_.y() << ", " << orientation_.z()
        << " ]";
    return out;
}