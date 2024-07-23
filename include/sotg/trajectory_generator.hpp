
#pragma once

#include <memory>

#include "sotg/constant_acceleration_solver.hpp"
#include "sotg/kinematic_solver.hpp"
#include "sotg/logger.hpp"
#include "sotg/path.hpp"
#include "sotg/path_manager.hpp"
#include "sotg/point.hpp"
#include "sotg/section.hpp"
#include "sotg/section_constraint.hpp"
#include "sotg/segment_constraint.hpp"

namespace SOTG {

// The enty point for interactions with SOTG in the form of new input or position and velocity calculations for a
// specifc point in time
class TrajectoryGenerator {
private:
    std::unique_ptr<detail::PathManager> path_manager_;
    std::shared_ptr<detail::KinematicSolver> kinematic_solver_;

    std::vector<std::map<std::string, double>> debug_info_vec_;
    const Logger& logger_;

public:
    TrajectoryGenerator();
    TrajectoryGenerator(const Logger& logger);

    double getDuration();
    void calcPositionAndVelocity(double time, Point& pos, Point& vel, int& id, bool disable_blending = false);
    void resetPath(Path path, std::vector<SectionConstraint> section_constraints,
                   std::vector<SegmentConstraint> segment_constraints);

    std::vector<std::map<std::string, double>>& getDebugInfo() { return debug_info_vec_; };
};
}  // namespace SOTG