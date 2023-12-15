#include "sotg/path_manager.hpp"

#include <iostream>

using namespace SOTG;
using namespace detail;

PathManager::PathManager(std::shared_ptr<KinematicSolver> solver_ptr,
                         std::vector<std::map<std::string, double>>& debug_info_vec_tg)
    : kinematic_solver_(solver_ptr)
    , debug_info_vec_(debug_info_vec_tg)
{
}

void PathManager::resetSections()
{
    sections_.clear();

    size_t num_sections = path_.getNumWaypoints() - 1;
    for (size_t i = 0; i < num_sections; i++) {
        Section section
            = kinematic_solver_->calcSection(path_.getPoint(i), path_.getPoint(i + 1), section_constraints_[i], i);

        sections_.push_back(section);
    }

    double current_time = 0.0;
    for (Section& section : sections_) {
        double duration = section.getDuration();
        section.setStartTime(current_time);
        current_time += duration;

#ifdef DEBUG
        std::cout << "Section start time: " << section.getStartTime() << ", duration: " << section.getDuration()
                  << std::endl;
#endif
    }
}

void PathManager::generateBlendSegments()
{
    // Iterate over all corners that could be blended
    // std::iterator is neccesary because of std::list
    // If sections was a vector instead of a list the code would look like this:
    //
    // for( size_t i = 1; i < sections.size(); ++i)
    // {
    //      ...
    //      Segment blend_segment = kinematic_solver_->calcSegment(sections[i-1], sections[i], ...);
    //      ...
    // }

    int blend_corner_index = 0;
    size_t blend_segment_id = 1;
    std::list<Section>::iterator it = sections_.begin();
    Section* last_section_addr = &(*it);
    for (++it; it != sections_.end(); ++it) {
        SegmentConstraint segment_constraint = segment_constraints_[blend_corner_index];

        Section* current_section_addr = &(*it);
        std::map<std::string, double> debug_info;
        std::shared_ptr<BlendSegment> blend_segment = kinematic_solver_->calcBlendSegment(
            *last_section_addr, *current_section_addr, segment_constraint, blend_segment_id, debug_info);
        last_section_addr = current_section_addr;
        ++blend_corner_index;
        blend_segment_id += 2;  // There is always one linear Segment in between

        segments_.push_back(blend_segment);
        debug_info_vec_.push_back(debug_info);
    }
}

void PathManager::generateLinearSegments()
{
    if (segments_.size() < 1) {
        std::shared_ptr<LinearSegment> single_segment(
            new LinearSegment(sections_.front(), sections_.front().getEndTime(), 0.0));
        single_segment->setID(0);

        segments_.push_back(single_segment);
    } else {
        size_t lin_segment_id = 0;
        std::list<std::shared_ptr<Segment>>::iterator it_segments;

        // Generate linear Segments inbetween the already existing blend segments, if any exist
        double last_t_end = 0;
        for (it_segments = segments_.begin(); it_segments != segments_.end(); ++it_segments) {
            std::shared_ptr<Segment> blend_segment = *it_segments;
            Section& pre_section = blend_segment->getPreBlendSection();

            double duration = blend_segment->getStartTime() - last_t_end;
            double t_start = last_t_end;
            std::shared_ptr<LinearSegment> segment(new LinearSegment(pre_section, duration, t_start));

            last_t_end = blend_segment->getEndTime();

            segment->setID(lin_segment_id);
            lin_segment_id += 2;  // There is always one blend Segment in between

            segments_.insert(it_segments, segment);
        }

        // Generate the last linear Segment
        it_segments = std::prev(segments_.end());
        std::shared_ptr<Segment> last_blend_segment = *it_segments;
        Section& last_section = last_blend_segment->getPostBlendSection();

        double t_end_without_shift = last_section.getEndTime();
        double t_end_with_shift = t_end_without_shift - last_section.getTimeShift();
        double duration = t_end_with_shift - last_blend_segment->getEndTime();

        std::shared_ptr<LinearSegment> last_segment(new LinearSegment(last_section, duration, last_t_end));

        last_segment->setID(lin_segment_id);

        segments_.push_back(last_segment);

#ifdef DEBUG
        for (auto& segment : segments_) {
            std::cout << "Segment " << segment->getID() << " start time: " << segment->getStartTime()
                      << ", duration: " << segment->getDuration() << std::endl;
        }
#endif
    }
}

void PathManager::resetSegments()
{
    segments_.clear();

    generateBlendSegments();

    generateLinearSegments();
}

void PathManager::resetPath(Path new_path, std::vector<SectionConstraint> new_section_constraints,
                            std::vector<SegmentConstraint> new_segment_constraints)
{
    path_ = new_path;
    segment_constraints_ = new_segment_constraints;
    section_constraints_ = new_section_constraints;

    if (path_.getNumWaypoints() != section_constraints_.size() + 1) {
        std::runtime_error("PathPlanner: Wrong amount of constrains to path, "
                           + std::to_string(section_constraints_.size()) + " constraints where given, but "
                           + std::to_string(path_.getNumWaypoints()) + " are needed");
    }

    resetSections();

    resetSegments();
}

// std::ostream& PathManager::operator<<(std::ostream& out)
// {
//     out << "[ ";

//     if (!sections_.empty()) {
//         std::list<Section>::iterator it;
//         for (it = sections_.begin(); it != std::prev(sections_.end()); ++it) {
//             out << it->getStartPoint() << ", ";
//         }
//         out << sections_.back().getStartFrame() << " ]";
//     } else {
//         out << " ]";
//     }

//     return out;
// }

const Segment& PathManager::getSegmentAtTime(double time)
{
    double last_t_end = 0.0;
    for (std::shared_ptr<Segment> segment : segments_) {
        double t_end = segment->getStartTime() + segment->getDuration();
        if (time > last_t_end && (time < t_end || (utility::nearlyEqual(time, t_end, 1e-6)))) {
            return *segment;
        }
        last_t_end = t_end;
    }
    if (utility::nearlyEqual(time, 0.0, 1e-6)) {
        return *(segments_.front());
    }

    throw std::runtime_error("Requested time \"" + std::to_string(time)
                             + "\" could not be mapped to any segment!");
}

const Section& PathManager::getSectionAtTime(double time)
{
    double last_t_end = 0.0;
    for (const Section& section : sections_) {
        // Explanation for time_shift in the Section class
        double time_shift = section.getTimeShift();
        double t_end = section.getStartTime() + section.getDuration() - time_shift;
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