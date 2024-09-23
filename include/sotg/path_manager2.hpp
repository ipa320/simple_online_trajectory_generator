#pragma once

#include <iterator>
#include <list>
#include <memory>
#include <vector>

#include "sotg/kinematic_solver2.hpp"
#include "sotg/path2.hpp"
#include "sotg/section2.hpp"

namespace SOTG {
namespace detail {

    // Stores and maintains the input path aswell as all generated sections and
    // segments
    class PathManager2 {
    private:
        Path2 path_;
        std::list<Section2> sections_;

        std::shared_ptr<KinematicSolver2> kinematic_solver_;

        std::vector<std::map<std::string, double>>& debug_info_vec_;

        void resetSections();

    public:
        PathManager2(std::shared_ptr<KinematicSolver2> solver,
                     std::vector<std::map<std::string, double>>& debug_info_vec);

        void resetPath(Path2 path);

        int getNumSections() { return sections_.size(); }

        const std::list<Section2>& getSections() const { return sections_; }

        Section2& getSectionAtTime(double time);

        std::ostream& operator<<(std::ostream& out);
    };

}  // namespace detail

}  // namespace SOTG