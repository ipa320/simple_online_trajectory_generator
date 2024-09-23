#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include <bits/stdc++.h>

#include "sotg/kinematic_solver2.hpp"
#include "sotg/section2.hpp"
#include "sotg/utility_functions.hpp"

#include "sotg/result.hpp"

namespace SOTG {
namespace detail {

    // Implements the logic for section and segment generation aswell as the
    // calculation of positions and velocities for specific points in time using a
    // bang coast bang profile
    class ConstantAccelerationSolver2 : public KinematicSolver2 {
    private:
        void calcPhaseDurationAndDistance(double a_max, double v_max, double L_total, Phase2Component& acc_phase_single_dof,
                                      Phase2Component& coast_phase_single_dof, Phase2Component& dec_phase_single_dof);

        void calcDurationAndDistancePerGroup(double& a_max, double& v_max, double distance,
                                               double& duration);

        void calcDurationAndDistance(Phase2& acc_phase, Phase2& coast_phase, Phase2& dec_phase,Point2& start_point, Point2& end_point,
                                         std::vector<double>& total_time_per_dof,
                                         std::vector<double>& total_length_per_dof, std::map<std::string, double>& group_acc, std::map<std::string, double>& group_vel);

        void calcPosAndVelSingleGroup(double section_length, const Phase2& phase,
                                          double phase_distance_to_p_start, double t_phase, double a_max_reduced,
                                          double v_max_reduced, double& pos_magnitude,
                                          double& vel_magnitude) const;

    public:
        ConstantAccelerationSolver2(SymbolGroupMap& symbol_map)
            : KinematicSolver2(symbol_map)
        {
        }

        Section2 calcSection(Point2& p_start_ref, Point2& p_end_ref, size_t section_id) override;

        void calcPosAndVelSection(double t_section, Section2& section, Result& result) const override;
    };
}  // namespace detail
}  // namespace SOTG