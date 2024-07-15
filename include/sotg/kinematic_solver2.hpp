#pragma once

#include <map>
#include <memory>

#include "sotg/section2.hpp"

namespace SOTG {
namespace detail {
    // Derived classes of this base class implement the logic for section and
    // segment generation aswell as the calculation of position and velocity for a
    // specific point in time
    class KinematicSolver2 {
    protected:
        SymbolGroupMap& symbol_groups_;

    public:
        virtual Section2 calcSection(Point2& p_start_ref, Point2& p_end_ref, size_t section_id) = 0;

        virtual void calcPosAndVelSection(double t_section, const Section2& section, Point2& pos,
                                          Point2& vel) const = 0;

        virtual ~KinematicSolver2() = default;
    };
}  // namespace detail
}  // namespace SOTG
