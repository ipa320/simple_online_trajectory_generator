#pragma once

#include <cmath>
#include <vector>

#include "sotg/phase.hpp"

namespace SOTG {
namespace detail {

    enum PhaseType { ConstantAcceleration, ConstantVelocity, ConstantDeacceleration };

    inline const char* ToString(PhaseType type)
    {
        switch (type) {
        case ConstantAcceleration:
            return "ConstantAcceleration";
        case ConstantVelocity:
            return "ConstantVelocity";
        case ConstantDeacceleration:
            return "ConstantDeacceleration";
        default:
            return "[Unknown Phase Type]";
        }
    }

    struct PhaseDoF {
        double duration = 0.0;
        double length = 0.0;
        double distance_start = 0.0;
        double angular_distance_start = 0.0;
    };

    // A specific kinematic state that applies to a specific part of a section
    struct Phase {
        std::vector<PhaseDoF> components;

        double duration = 0.0;
        double length = 0.0;

        double t_start = 0.0;
        double distance_start = 0.0;

        PhaseType type;
    };
}  // namespace detail
}  // namespace SOTG