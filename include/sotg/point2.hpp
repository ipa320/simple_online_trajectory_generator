
#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/symbol_group.hpp"
#include "sotg/utility_functions.hpp"
#include "sotg/value_group.hpp"

namespace SOTG {

// Wraps std::vector and allows vector arithmetic, stores information about the
// unit of a particular value
class Point2 {
private:
    ValueGroupMap value_groups_;
    std::string name_ = "ND";

public:
    Point2(SymbolGroupMap symbol_map, std::string name = "ND");

    std::string getName() { return name_; }

    ValueGroup& operator[](std::string key) { return value_groups_[key]; }

    friend std::ostream& operator<<(std::ostream& out, const Point2& point)
    {
        out << "[[ " << point.name_ << " ] {";
        for (auto& [key, value] : point.value_groups_) {
            out << " " << key << " : " << value << ",";
        }
        out << "\b";
        out << " }]";

        return out;
    }
};

}  // namespace SOTG