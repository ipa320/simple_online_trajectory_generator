
#pragma once

#include <memory>

#include "sotg/constant_acceleration_solver2.hpp"
#include "sotg/kinematic_solver2.hpp"
#include "sotg/path2.hpp"
#include "sotg/path_manager.hpp"
#include "sotg/path_manager2.hpp"
#include "sotg/point2.hpp"
#include "sotg/section2.hpp"
#include "sotg/symbol_group.hpp"
#include "sotg/result.hpp"

namespace SOTG {

// The enty point for interactions with SOTG in the form of new input or position and velocity calculations for a
// specifc point in time
class TrajectoryGenerator2 {
private:
    SymbolGroupMap symbol_group_map_;

    std::unique_ptr<detail::PathManager2> path_manager_;
    std::shared_ptr<detail::KinematicSolver2> kinematic_solver_;

    std::vector<std::map<std::string, double>> debug_info_vec_;

public:
    TrajectoryGenerator2(SymbolGroupMap symbol_groups);

    double getDuration();
    void calcPositionAndVelocity(double time, Result& result);
    void resetPath(Path2 path);

    std::vector<std::map<std::string, double>>& getDebugInfo() { return debug_info_vec_; };
};
}  // namespace SOTG