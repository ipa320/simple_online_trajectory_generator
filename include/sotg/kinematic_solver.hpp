#pragma once

#include <map>
#include <memory>
#include <string>

#include "sotg/blend_segment.hpp"
#include "sotg/linear_segment.hpp"
#include "sotg/logger.hpp"
#include "sotg/section.hpp"
#include "sotg/segment.hpp"

namespace SOTG {
namespace detail {
    // Derived classes of this base class implement the logic for section and
    // segment generation aswell as the calculation of position and velocity for a
    // specific point in time
    class KinematicSolver {
    protected:
        const Logger& logger_;

    public:
        KinematicSolver(const Logger& logger)
            : logger_(logger)
        {
        }

        virtual Section calcSection(Point& p_start_ref, Point& p_end_ref, SectionConstraint constraint_copy,
                                    size_t section_id)
            = 0;
        virtual std::shared_ptr<BlendSegment>
        calcBlendSegment(Section& pre_section, Section& post_section, const SegmentConstraint& constraint,
                         size_t segment_id, std::map<std::string, double>& debug_output)
            = 0;

        virtual void calcPosAndVelSection(double t_section, const Section& section, Point& pos,
                                          Point& vel) const = 0;

        virtual void calcPosAndVelLinearSegment(double t_section, const LinearSegment& segment, Point& pos,
                                                Point& vel) const = 0;

        virtual void calcPosAndVelBlendSegment(double t_segment, const BlendSegment& segment, Point& pos,
                                               Point& vel) const = 0;

        virtual ~KinematicSolver() = default;
    };
}  // namespace detail
}  // namespace SOTG
