#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include <bits/stdc++.h>

#include "sotg/blend_segment.hpp"
#include "sotg/kinematic_solver.hpp"
#include "sotg/linear_segment.hpp"
#include "sotg/section.hpp"
#include "sotg/segment_constraint.hpp"
#include "sotg/utility_functions.hpp"

namespace SOTG {
namespace detail {

    // Implements the logic for section and segment generation aswell as the
    // calculation of positions and velocities for specific points in time using a
    // bang coast bang profile
    class ConstantAccelerationSolver : public KinematicSolver {
    private:
        size_t current_section_id{0};
        size_t current_segment_id{0};

        void calcSegmentPreparations(const Section& pre_section, const Section& post_section,
                                     std::vector<double>& a_max_post, std::vector<double>& a_max_pre,
                                     double& L_acc_magnitude_post, double& T_acc_post, double& T_acc_pre);

        void calcAccAndVelPerDoF(const Section& section, std::vector<double>& a_max_vec,
                                 std::vector<double>& v_max_vec);

        void calcPhaseTimeAndDistance(double& a_max, double& v_max, double L_total, PhaseDoF& acc_phase_single_dof,
                                      PhaseDoF& coast_phase_single_dof, PhaseDoF& dec_phase_single_dof);

        void calcTotalTimeAndDistanceSingleDoF(double& a_max, double& v_max, double total_length,
                                               double& total_time, size_t coordinate_id);

        void calcTimesAndLengthsMultiDoF(Phase& acc_phase, Phase& coast_phase, Phase& dec_phase,
                                         std::vector<double>& total_time_per_dof,
                                         std::vector<double>& total_length_per_dof, Point diff,
                                         std::vector<double>& a_max_vec, std::vector<double>& v_max_vec);

        void calcSecondBlendingDist(double T_blend, double T_acc_post, double a_max_magnitude_post,
                                    double vel_pre_blend_magnitude, double blending_dist_pre,
                                    double& blending_dist_post, size_t segment_id);

        void calcPosAndVelSingleDoFLinear(double section_length, const Phase& phase,
                                          double phase_distance_to_p_start, double t_phase, double a_max_reduced,
                                          double v_max_reduced, double& pos_magnitude,
                                          double& vel_magnitude) const;
        void calcVelAndTimeByDistance(const Section& section, double distance, Point& velocity_per_dof,
                                      double& time_when_distance_is_reached);

        void calcPreBlendParams(double blending_dist_pre, const Section& pre_section, Point& A_blend,
                                double& T_blend, double& vel_pre_blend_magnitude,
                                double& absolute_blend_start_time);

        void calcPostBlendParams(double blending_dist_pre, const Section& pre_section, Point& C_blend,
                                 double& T_blend, double& vel_pre_blend_magnitude, double& t_abs_start_blend);

        bool isBlendAccelerationTooHigh(const std::vector<double>& a_max, const double& T_blend,
                                        const double& vel_pre_blend_magnitude,
                                        const double& vel_post_blend_magnitude, const Section& pre_section,
                                        const Section& post_section, size_t segment_id);
        void setNoBlendingParams(const Section& pre_section, const Section& post_section, double& T_blend,
                                 double& absolute_blend_start_time_with_shift,
                                 double& absolute_blend_end_time_without_shift, Point& A_blend, Point& C_blend,
                                 double& vel_pre_blend_magnitude, double& vel_post_blend_magnitude);

    public:
        ConstantAccelerationSolver(const Logger& logger)
            : KinematicSolver(logger)
        {
        }

        Section calcSection(Point& p_start_ref, Point& p_end_ref, SectionConstraint constraint_copy,
                            size_t section_id) override;
        std::shared_ptr<BlendSegment> calcBlendSegment(Section& pre_section, Section& post_section,
                                                       const SegmentConstraint& constraint, size_t segment_id,
                                                       std::map<std::string, double>& debug_output) override;

        void calcPosAndVelSection(double t_section, const Section& section, Point& pos, Point& vel) const override;

        void calcPosAndVelLinearSegment(double t_section, const LinearSegment& segment, Point& pos,
                                        Point& vel) const override;

        void calcPosAndVelBlendSegment(double t_segment, const BlendSegment& segment, Point& pos,
                                       Point& vel) const override;
    };
}  // namespace detail
}  // namespace SOTG