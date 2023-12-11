#pragma once

#include "sotg/section.hpp"
#include "sotg/segment.hpp"
#include "sotg/segment_constraint.hpp"

namespace SOTG {
namespace detail {

    class KinematicSolver;

    // A container that is used to hold the information necessary to calculate positions and velocities during its
    // duration Unlike sections, segments can be linear and blended, whereby linear segments behave like cropped
    // sections. Blend segments take two references to adjacent sections, allowing for the smooth transition
    // between the two.
    class BlendSegment : public Segment {
    private:
        Section& pre_section_;
        Section& post_section_;

        double pre_velocity_magnitude_;
        double post_velocity_magnitude_;

        SegmentConstraint constraint_;

    public:
        BlendSegment(Section& pre_section_ref, Section& post_section_ref, SegmentConstraint constraint_copy,
                     double pre_vel_mag, double post_vel_mag, double duration, double t_start)
            : pre_section_(pre_section_ref)
            , post_section_(post_section_ref)
        {
            constraint_ = constraint_copy;
            pre_velocity_magnitude_ = pre_vel_mag;
            post_velocity_magnitude_ = post_vel_mag;
            duration_ = duration;
            start_time_ = t_start;
        }

        void calcPosAndVel([[maybe_unused]] double t_section, [[maybe_unused]] double t_segment, Point& pos,
                           Point& vel, const KinematicSolver& solver) const override;

        const Point& getPreBlendDirection() const { return pre_section_.getDirection(); }
        const Point& getPostBlendDirection() const { return post_section_.getDirection(); }

        void setPreBlendVelocityMagnitude(double value) { pre_velocity_magnitude_ = value; }
        void setPostBlendVelocityMagnitude(double value) { post_velocity_magnitude_ = value; }

        double getPreBlendVelocityMagnitude() const { return pre_velocity_magnitude_; }
        double getPostBlendVelocityMagnitude() const { return post_velocity_magnitude_; }

        void setDuration(double duration) { duration_ = duration; }
        void setStartTime(double t_start) { start_time_ = t_start; }

        void setStartPoint(Point p) { start_point_ = p; }
        void setEndPoint(Point p) { end_point_ = p; }

        double getStartTime() const { return start_time_; }
        double getDuration() const { return duration_; }
        double getEndTime() const { return start_time_ + duration_; }

        const Point& getStartPoint() const { return start_point_; }
        const Point& getEndPoint() const { return end_point_; }

        Section& getPreBlendSection() const override { return pre_section_; };
        Section& getPostBlendSection() const override { return post_section_; };
    };
}  // namespace detail
}  // namespace SOTG