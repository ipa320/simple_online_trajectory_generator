#pragma once

#include <iterator>
#include <list>
#include <memory>
#include <vector>

#include "sotg/blend_segment.hpp"
#include "sotg/kinematic_solver.hpp"
#include "sotg/linear_segment.hpp"
#include "sotg/path.hpp"
#include "sotg/section.hpp"
#include "sotg/section_constraint.hpp"
#include "sotg/segment_constraint.hpp"

namespace SOTG {
namespace detail {

    // Stores and maintains the input path aswell as all generated sections and segments
    class PathManager {
    private:
        Path path_;
        std::list<Section> sections_;
        std::list<std::shared_ptr<Segment>> segments_;

        std::vector<SectionConstraint> section_constraints_;
        std::vector<SegmentConstraint> segment_constraints_;

        std::shared_ptr<KinematicSolver> kinematic_solver_;

        std::vector<std::map<std::string, double>>& debug_info_vec_;

        void resetSections();
        void resetSegments();
        void generateBlendSegments();
        void generateLinearSegments();

    public:
        PathManager(std::shared_ptr<KinematicSolver> solver,
                    std::vector<std::map<std::string, double>>& debug_info_vec);

        void resetPath(Path path, std::vector<SectionConstraint> section_constraints,
                       std::vector<SegmentConstraint> segment_constraints);

        int getNumSections() { return sections_.size(); }
        int getNumSegments() { return segments_.size(); }
        const std::list<std::shared_ptr<Segment>>& getSegments() const { return segments_; }
        const std::list<Section>& getSections() const { return sections_; }

        const Section& getSectionAtTime(double time);
        const Segment& getSegmentAtTime(double time);

        std::ostream& operator<<(std::ostream& out);
    };

}  // namespace detail

}  // namespace SOTG