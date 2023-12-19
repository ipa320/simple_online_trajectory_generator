#pragma once

#include <algorithm>
#include <vector>

#include "sotg/phase.hpp"
#include "sotg/pose.hpp"
#include "sotg/section_constraint.hpp"
#include "sotg/transform.hpp"

namespace SOTG {
namespace detail {

    // Connects to waypoints of a path and is limited by an instance of SectionConstraint
    // Stores kinematic states for position and velocity calculations along this sectionin inside of phases
    class Section {
    private:
        Pose& start_pose_;
        Pose& end_pose_;
        Transform transform_;
        double length_;
        SectionConstraint constraint_;

        std::vector<Phase> phases_;

        int index_slowest_dof_ = -1;

        std::vector<double> adapted_acceleration_;
        std::vector<double> adapted_velocity_;
        Eigen::VectorXd adapted_acc_max_linear_;
        Eigen::Vector3d adapted_acc_max_angular_;
        Eigen::VectorXd adapted_vel_max_linear_;
        Eigen::Vector3d adapted_vel_max_angular_;

        double duration_ = 0.0;
        double start_time_ = 0.0;

        size_t id_;

        // time_shift is the duration of the preceeding blend segment, without blending it is always zero
        double time_shift_ = 0.0;

    public:
        const Pose& getStartPose() const { return start_pose_; }
        const Pose& getEndPose() const { return end_pose_; }

        double getAccMaxLinear() const { return constraint_.getAccelerationMagnitudeLinear(); }
        double getAccMaxAngular() const { return constraint_.getAccelerationMagnitudeAngular(); }
        double getVelMaxLinear() const { return constraint_.getVelocityMagnitudeLinear(); }
        double getVelMaxAngular() const { return constraint_.getVelocityMagnitudeAngular(); }

        double getLength() const { return length_; }
        const Transform& getTransform() const { return transform_; }
        void setLength(double length) { length_ = length; }
        void setTransform(const Transform& transform) { transform_ = transform; }

        Section(Pose& p_start_ref, Pose& p_end_ref, SectionConstraint constraint_copy, size_t section_id);

        void setIndexSlowestDoF(int index) { index_slowest_dof_ = index; }

        void setDuration(double time) { duration_ = time; }
        double getDuration() const { return duration_; }

        const Phase& getPhaseByDistance(double distance) const;

        const std::vector<double>& getAdaptedAcceleration() const { return adapted_acceleration_; }
        const std::vector<double>& getAdaptedVelocity() const { return adapted_velocity_; }

        const Eigen::VectorXd& getAccMaxLinearAdapted() const { return adapted_acc_max_linear_; }
        const Eigen::Vector3d& getAccMaxAngularAdapted() const { return adapted_acc_max_angular_; }
        const Eigen::VectorXd& getVelMaxLinearAdapted() const { return adapted_vel_max_linear_; }
        const Eigen::Vector3d& getVelMaxAngularAdapted() const { return adapted_vel_max_angular_; }

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

        void setTimeShift(double shift) { time_shift_ = shift; }
        double getTimeShift() const { return time_shift_; }

        void setID(size_t new_id) { id_ = new_id; }
        size_t getID() { return id_; }
    };
}  // namespace detail
}  // namespace SOTG