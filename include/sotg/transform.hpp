#pragma once

#include "sotg/pose.hpp"

namespace SOTG {
namespace detail {

    // Wraps std::vector and allows vector arithmetic, stores information about the unit of a particular value
    class Transform : public Pose {
    private:
        double angular_distance_;

    public:
        Transform() { }
        Transform(const Eigen::VectorXd& loc, const Eigen::Quaterniond& rot, double angular_distance)
            : Pose(loc, rot)
            , angular_distance_(angular_distance)
        {
        }
        double getAngularDistance() const { return angular_distance_; }
    };

}  // namespace detail
}  // namespace SOTG