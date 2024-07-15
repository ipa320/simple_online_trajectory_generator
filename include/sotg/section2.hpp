#pragma once

#include <algorithm>
#include <vector>

#include "sotg/phase.hpp"
#include "sotg/point2.hpp"

namespace SOTG {
namespace detail {

    // Connects to waypoints of a path and is limited by an instance of SectionConstraint
    // Stores kinematic states for position and velocity calculations along this sectionin inside of phases
    class Section2 {
    private:
        Point2& start_point_;
        Point2& end_point_;

        SymbolGroupMap& symbol_groups_;

        double length_;

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
        const Point2& getStartPoint() const { return start_point_; }
        const Point2& getEndPoint() const { return end_point_; }

        double getLength() const { return length_; }
        // const Point2& getDirection() const { return dir_; }
        // const Point2& getDifference() const { return diff_; }
        // void setLength(double length) { length_ = length; }
        // void setDirection(const Point2& dir) { dir_ = dir; }
        // void setDifference(const Point2& diff) { diff_ = diff; }

        Section2(Point2& p_start_ref, Point2& p_end_ref, size_t section_id)
            : start_point_(p_start_ref)
            , end_point_(p_end_ref)
            , symbol_groups_(p_start_ref.getSymbolGroups())
            , id_(section_id)
        {
        }

        void setIndexSlowestDoF(int index) { index_slowest_dof_ = index; }

        void setDuration(double time) { duration_ = time; }
        double getDuration() const { return duration_; }

        const Phase& getPhaseByDistance(double distance) const;

        const std::vector<double>& getAdaptedAcceleration() const { return adapted_acceleration_; }
        const std::vector<double>& getAdaptedVelocity() const { return adapted_velocity_; }
        void setAdaptedAcceleration(const std::vector<double>& new_adapted_acceleration)
        {
            adapted_acceleration_ = new_adapted_acceleration;
        }
        void setAdaptedVelocity(const std::vector<double>& new_adapted_velocity)
        {
            adapted_velocity_ = new_adapted_velocity;
        }

        const Phase& getPhaseByTime(double time) const;
        void setPhases(const std::vector<Phase>& new_phases) { phases_ = new_phases; }
        const std::vector<Phase>& getPhases() const { return phases_; }

        double getStartTime() const { return start_time_; }
        double getEndTime() const { return start_time_ + duration_; }
        void setStartTime(double new_start_time) { start_time_ = new_start_time; }

        int getIndexSlowestDoF() const { return index_slowest_dof_; }

        const Phase& getPhaseByType(PhaseType type) const;

        void setID(size_t new_id) { id_ = new_id; }
        size_t getID() { return id_; }
    };
}  // namespace detail
}  // namespace SOTG