#include "sotg/path_manager2.hpp"

#include <iostream>

using namespace SOTG;
using namespace detail;

PathManager2::PathManager2(std::shared_ptr<KinematicSolver2> solver_ptr,
                           std::vector<std::map<std::string, double>>& debug_info_vec_tg)
    : kinematic_solver_(solver_ptr)
    , debug_info_vec_(debug_info_vec_tg)
{
}

void PathManager2::resetSections()
{
    sections_.clear();

    size_t num_sections = path_.getNumWaypoints() - 1;
    for (size_t i = 0; i < num_sections; i++) {
        Section2 section = kinematic_solver_->calcSection(path_.getPoint(i), path_.getPoint(i + 1), i);

        sections_.push_back(section);
    }

    double current_time = 0.0;
    for (Section2& section : sections_) {
        double duration = section.getDuration();
        section.setStartTime(current_time);
        current_time += duration;

#ifdef DEBUG
        std::cout << "Section2 start time: " << section.getStartTime() << ", duration: " << section.getDuration()
                  << std::endl;
#endif
    }
}

void PathManager2::resetPath(Path2 new_path)
{
    path_ = new_path;
    std::cout << "Test";
    resetSections();
}

std::ostream& PathManager2::operator<<(std::ostream& out)
{
    out << "[ ";

    if (!sections_.empty()) {
        std::list<Section2>::iterator it;
        for (it = sections_.begin(); it != std::prev(sections_.end()); ++it) {
            out << it->getStartPoint() << ", ";
        }
        out << sections_.back().getStartPoint() << " ]";
    } else {
        out << " ]";
    }

    return out;
}

const Section2& PathManager2::getSectionAtTime(double time)
{
    double last_t_end = 0.0;
    for (const Section2& section : sections_) {
        // Explanation for time_shift in the Section2 class
        // double time_shift = section.getTimeShift();
        double t_end = section.getStartTime() + section.getDuration();  // - time_shift;
        if (time > last_t_end && (time < t_end || (utility::nearlyEqual(time, t_end, 1e-6)))) {
            return section;
        }
        last_t_end = t_end;
    }
    if (utility::nearlyEqual(time, 0.0, 1e-6)) {
        return sections_.front();
    }

    throw std::runtime_error("Requested time \"" + std::to_string(time)
                             + "\" could not be mapped to any section!");
}