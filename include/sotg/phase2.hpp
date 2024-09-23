#pragma once

#include <cmath>
#include <vector>
#include <map>

namespace SOTG {
namespace detail {

    enum PhaseType2 { ConstantAcceleration2, ConstantVelocity2, ConstantDeacceleration2 };

    inline const char* ToString(PhaseType2 type)
    {
        switch (type) {
        case ConstantAcceleration2:
            return "ConstantAcceleration";
        case ConstantVelocity2:
            return "ConstantVelocity";
        case ConstantDeacceleration2:
            return "ConstantDeacceleration";
        default:
            return "[Unknown Phase Type]";
        }
    }

    struct Phase2Component {
        double duration = 0.0;
        double length = 0.0;
        double distance_p_start = 0.0;
    };

    // A specific kinematic state that applies to a specific part of a section
    struct Phase2 {
        std::map<std::string ,Phase2Component> components;

        double duration = 0.0;
        double length = 0.0;

        double t_start = 0.0;
        double distance_p_start = 0.0;

        PhaseType2 type;
    };
}  // namespace detail
}  // namespace SOTG