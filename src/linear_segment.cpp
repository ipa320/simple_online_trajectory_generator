#include "sotg/linear_segment.hpp"

#include "sotg/kinematic_solver.hpp"
using namespace SOTG;
using namespace detail;

void LinearSegment::calcPosAndVel(double t_section, [[maybe_unused]] double t_segment, Pose& pos, Pose& vel,
                                  const KinematicSolver& solver) const
{
    solver.calcPosAndVelLinearSegment(t_section, *this, pos, vel);
}