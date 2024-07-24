#include "sotg/trajectory_generator.hpp"

#include <iostream>
#include <memory>

using namespace SOTG;
using namespace detail;

TrajectoryGenerator::TrajectoryGenerator()
    : default_logger_(new Logger())
    , logger_(*default_logger_)
    , kinematic_solver_(new detail::ConstantAccelerationSolver(logger_))
{
    path_manager_
        = std::unique_ptr<detail::PathManager>(new detail::PathManager(kinematic_solver_, debug_info_vec_));
}

TrajectoryGenerator::TrajectoryGenerator(const Logger& logger)
    : logger_(logger)
    , kinematic_solver_(new detail::ConstantAccelerationSolver(logger_))
{
    path_manager_
        = std::unique_ptr<detail::PathManager>(new detail::PathManager(kinematic_solver_, debug_info_vec_));
}

void TrajectoryGenerator::resetPath(Path path, std::vector<SectionConstraint> section_constraints,
                                    std::vector<SegmentConstraint> segment_constraints)
{
    path_manager_->resetPath(path, section_constraints, segment_constraints);
}

double TrajectoryGenerator::getDuration()
{
    double total_time = 0.0;
    for (std::shared_ptr<Segment> segment : path_manager_->getSegments()) {
        total_time += segment->getDuration();
    }
    return total_time;
}

void TrajectoryGenerator::calcPositionAndVelocity(double time, Point& pos, Point& vel, int& id,
                                                  bool disable_blending)
{
    if (disable_blending) {
        const Section& section = path_manager_->getSectionAtTime(time);

        double t_section = time - section.getStartTime();

        kinematic_solver_->calcPosAndVelSection(t_section, section, pos, vel);
    } else {
        const Section& section = path_manager_->getSectionAtTime(time);
        const Segment& segment = path_manager_->getSegmentAtTime(time);

        double t_segment = time - segment.getStartTime();

        // t_section needs to be time shifted because the start times for the section where calculated without
        // knowing how much time would be saved by blending between sections
        double t_section = time - (section.getStartTime() - section.getTimeShift());

        segment.calcPosAndVel(t_section, t_segment, pos, vel, *kinematic_solver_);

        id = segment.getID();
    }
}