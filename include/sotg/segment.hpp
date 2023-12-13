#pragma once

#include "sotg/section.hpp"

namespace SOTG {
namespace detail {

    class KinematicSolver;
    // A container that is used to hold the information necessary to calculate positions and velocities during its
    // duration Unlike sections, segments can be linear and blended, whereby linear segments behave like cropped
    // sections. Blend segments take two references to adjacent sections, allowing for the smooth transition
    // between the two.
    class Segment {
    protected:
        Segment(double duration, double t_start)
            : duration_(duration)
            , start_time_(t_start)
            , id_(-1)
        {
        }

        double duration_;
        double start_time_;

        Point start_point_;
        Point end_point_;

        int id_;

    public:
        virtual void calcPosAndVel([[maybe_unused]] double t_section, [[maybe_unused]] double t_segment,
                                   Point& pos, Point& vel, const KinematicSolver& solver) const = 0;

        void setDuration(double duration) { duration_ = duration; }
        void setStartTime(double t_start) { start_time_ = t_start; }

        void setStartPoint(const Point& p) { start_point_ = p; }
        void setEndPoint(const Point& p) { end_point_ = p; }

        void setID(int num) { id_ = num; }

        double getStartTime() const { return start_time_; }
        double getDuration() const { return duration_; }
        double getEndTime() const { return start_time_ + duration_; }

        const Point& getStartPoint() const { return start_point_; }
        const Point& getEndPoint() const { return end_point_; }

        int getID() const { return id_; }

        virtual Section& getPreBlendSection() const
        {
            throw std::runtime_error("Called getPreBlendSection from LinearSegment");
        };
        virtual Section& getPostBlendSection() const
        {
            throw std::runtime_error("Called getPostBlendSection from LinearSegment");
        };

        virtual Section& getSection() const { throw std::runtime_error("Called getSection from BlendSegment"); };

        virtual ~Segment() = default;
    };
}  // namespace detail
}  // namespace SOTG