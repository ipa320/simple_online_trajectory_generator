#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include <bits/stdc++.h>

#include "sotg/kinematic_solver2.hpp"
#include "sotg/section2.hpp"
#include "sotg/utility_functions.hpp"

namespace SOTG {
namespace detail {

    // Implements the logic for section and segment generation aswell as the
    // calculation of positions and velocities for specific points in time using a
    // bang coast bang profile
    class ConstantAccelerationSolver2 : public KinematicSolver2 {
    private:
        void calcAccAndVelPerDoF(const Section2& section, std::vector<double>& a_max_vec,
                                 std::vector<double>& v_max_vec);

        void calcPhaseTimeAndDistance(double& a_max, double& v_max, double L_total, PhaseDoF& acc_phase_single_dof,
                                      PhaseDoF& coast_phase_single_dof, PhaseDoF& dec_phase_single_dof);

        void calcTotalTimeAndDistanceSingleDoF(double& a_max, double& v_max, double total_length,
                                               double& total_time);

        void calcTimesAndLengthsMultiDoF(Phase& acc_phase, Phase& coast_phase, Phase& dec_phase,
                                         std::vector<double>& total_time_per_dof,
                                         std::vector<double>& total_length_per_dof, Point2 diff,
                                         std::vector<double>& a_max_vec, std::vector<double>& v_max_vec);

        void calcPosAndVelSingleDoFLinear(double section_length, const Phase& phase,
                                          double phase_distance_to_p_start, double t_phase, double a_max_reduced,
                                          double v_max_reduced, double& pos_magnitude,
                                          double& vel_magnitude) const;

    public:
        ConstantAccelerationSolver2(SymbolGroupMap& symbol_groups)
            : symbol_groups_(symbol_groups)
        {
        }

        Section2 calcSection(Point2& p_start_ref, Point2& p_end_ref, size_t section_id) override;

        void calcPosAndVelSection(double t_section, const Section2& section, Point2& pos,
                                  Point2& vel) const override;
    };
}  // namespace detail
}  // namespace SOTG