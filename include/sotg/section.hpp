#pragma once

#include <vector>

#include "sotg/phase.hpp"
#include "sotg/point.hpp"
#include "sotg/section_constraint.hpp"

namespace SOTG {
namespace detail {

    // Connects to waypoints of a path and is limited by an instance of SectionConstraint
    // Stores kinematic states for position and velocity calculations along this sectionin inside of phases
    class Section {
    private:
        Point& start_point_;
        Point& end_point_;
        Point diff_;
        Point dir_;
        double length_;
        SectionConstraint constraint_;

        std::vector<Phase> phases_;

        int index_slowest_dof_ = -1;

        std::vector<double> adapted_acceleration_;
        std::vector<double> adapted_velocity_;

        double duration_ = 0.0;
        double start_time_ = 0.0;

        size_t id_;

        // time_shift is the duration of the preceeding blend segment, without blending it is always zero
        double time_shift_ = 0.0;

    public:
        const Point& getStartPoint() const { return start_point_; }
        const Point& getEndPoint() const { return end_point_; }

        double getAccMaxLinear() const { return constraint_.getAccelerationMagnitudeLinear(); }
        double getAccMaxAngular() const { return constraint_.getAccelerationMagnitudeAngular(); }
        double getVelMaxLinear() const { return constraint_.getVelocityMagnitudeLinear(); }
        double getVelMaxAngular() const { return constraint_.getVelocityMagnitudeAngular(); }

        double getLength() const { return length_; }
        const Point& getDirection() const { return dir_; }
        const Point& getDifference() const { return diff_; }
        void setLength(double length) { length_ = length; }
        void setDirection(Point dir) { dir_ = dir; }
        void setDifference(Point diff) { diff_ = diff; }

        Section(Point& p_start_ref, Point& p_end_ref, SectionConstraint constraint_copy, size_t section_id);

        void setIndexSlowestDoF(int index) { index_slowest_dof_ = index; }

        void setDuration(double time) { duration_ = time; }
        double getDuration() const { return duration_; }

        const Phase& getPhaseByDistance(double distance) const;

        const std::vector<double>& getAdaptedAcceleration() const { return adapted_acceleration_; }
        const std::vector<double>& getAdaptedVelocity() const { return adapted_velocity_; }
        void setAdaptedAcceleration(std::vector<double> new_adapted_acceleration)
        {
            adapted_acceleration_ = new_adapted_acceleration;
        }
        void setAdaptedVelocity(std::vector<double> new_adapted_velocity)
        {
            adapted_velocity_ = new_adapted_velocity;
        }

        const Phase& getPhaseByTime(double time) const;
        void setPhases(std::vector<Phase> new_phases) { phases_ = new_phases; }
        const std::vector<Phase>& getPhases() const { return phases_; }

        double getStartTime() const { return start_time_; }
        double getEndTime() const { return start_time_ + duration_; }
        void setStartTime(double new_start_time) { start_time_ = new_start_time; }

        int getIndexSlowestDoF() const { return index_slowest_dof_; }

        const Phase& getPhaseByType(PhaseType type) const;

        void setTimeShift(double shift) { time_shift_ = shift; }
        double getTimeShift() const { return time_shift_; }

        void setID(size_t new_id) { id_ = new_id; }
        size_t getID() { return id_; }
    };
}  // namespace detail
}  // namespace SOTG