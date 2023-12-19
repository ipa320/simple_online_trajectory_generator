#include "sotg/blend_segment.hpp"

#include "sotg/kinematic_solver.hpp"
#include "sotg/pose.hpp"
using namespace SOTG;
using namespace detail;

void BlendSegment::calcPosAndVel([[maybe_unused]] double t_section, double t_segment, Pose& pos, Pose& vel,
                                 const KinematicSolver& solver) const
{
    // solver.calcPosAndVelBlendSegment(t_segment, *this, pos, vel);
}