#include "sotg/trajectory_generator2.hpp"

#include <iostream>
#include <memory>

using namespace SOTG;
using namespace detail;

TrajectoryGenerator2::TrajectoryGenerator2(SymbolGroupMap symbol_groups)
    : symbol_group_map_(symbol_groups)
    , kinematic_solver_(new detail::ConstantAccelerationSolver2(symbol_group_map_))
{
    path_manager_
        = std::unique_ptr<detail::PathManager2>(new detail::PathManager2(kinematic_solver_, debug_info_vec_));
}

void TrajectoryGenerator2::resetPath(Path2 path) { path_manager_->resetPath(path); }

void TrajectoryGenerator2::calcPositionAndVelocity(double time, Point2& pos, Point2& vel, int& id,
                                                   bool disable_blending)
{
    const Section2& section = path_manager_->getSectionAtTime(time);

    double t_section = time - section.getStartTime();

    kinematic_solver_->calcPosAndVelSection(t_section, section, pos, vel);
}