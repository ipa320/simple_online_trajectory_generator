#pragma once

#include "sotg/segment.hpp"

namespace SOTG {
namespace detail {

    class KinematicSolver;

    // A container that is used to hold the information necessary to calculate
    // positions and velocities during its duration Unlike sections, segments can be
    // linear and blended, whereby linear segments behave like cropped sections.
    // Blend segments take two references to adjacent sections, allowing for the
    // smooth transition between the two.
    class LinearSegment : public Segment {
    private:
        Section& section_;

    public:
        LinearSegment(Section& section, double duration, double t_start)
            : Segment(duration, t_start)
            , section_(section)
        {
        }

        void calcPosAndVel([[maybe_unused]] double t_section, [[maybe_unused]] double t_segment, Pose& pos,
                           Pose& vel, const KinematicSolver& solver) const override;

        void setDuration(double duration) { duration_ = duration; }
        void setStartTime(double t_start) { start_time_ = t_start; }

        void setStartPoint(const Pose& p) { start_point_ = p; }
        void setEndPoint(const Pose& p) { end_point_ = p; }

        double getStartTime() const { return start_time_; }
        double getDuration() const { return duration_; }
        double getEndTime() const { return start_time_ + duration_; }

        const Pose& getStartPoint() const { return start_point_; }
        const Pose& getEndPoint() const { return end_point_; }

        Section& getSection() const override { return section_; }
    };
}  // namespace detail
}  // namespace SOTG