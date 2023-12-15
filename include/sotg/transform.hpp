#pragma once

#include "sotg/point.hpp"

namespace SOTG {
namespace detail {

    // Wraps std::vector and allows vector arithmetic, stores information about the unit of a particular value
    class Transform : public Frame {
    private:
        double angular_distance_;

    public:
        Transform(double angular_distance)
            : angular_distance_(angular_distance)
        {
        }
    };

}  // namespace detail
}  // namespace SOTG