#pragma once

#include <initializer_list>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sotg/symbol_group.hpp"

namespace SOTG {

// The Blueprint for are SymbolGroupValue object.
// SymbolGroups are used in a SymbolMap to define the structure of the Poses that the trajectorie should cover.
// A SymbolGroup defines part of a Pose that share the same unit, e.g. Position in metres or Euler Angles in
// radians Exemple: a SymbolGroup could define two Positions x,y that are jointly limited in accelration and
// velocity, together with another SymbolGroup that defines a single Euler Angle that has its own angular
// acceleration and velocity limits, forms the basis for a 2D trajectory.

class ValueGroup {
private:
    SymbolGroup symbols_;

    Eigen::VectorXd values_;
    Eigen::Quaterniond quat_value_;

public:
    ValueGroup(SymbolGroup symbol_group)
        : symbols_(symbol_group)
    {
        values_ = Eigen::VectorXd{symbols_.size()};
    }
    ValueGroup() { }

    void operator=(std::initializer_list<double> list);
    double& operator[](std::string key);
    ValueGroup operator+(ValueGroup& value_group);
    // ValueGroup operator-(ValueGroup& value_group);
    // ValueGroup operator*(double scalar);

    // should not be accessable
    void setSymbols(SymbolGroup symbol_group) { symbols_ = symbol_group; }
    const SymbolGroup& getSymbols() const { return symbols_; }

    bool is_quaternion = false;

    friend std::ostream& operator<<(std::ostream& out, const ValueGroup& vg)
    {
        auto& symbols = vg.getSymbols();
        out << "{";
        for (size_t i = 0; i < vg.getSymbols().size(); i++) {
            out << " " << symbols[i] << " : " << std::to_string(vg.values_[i]) << ",";
        }
        out << "\b";
        out << " }";

        return out;
    }
};

using ValueGroupMap = std::unordered_map<std::string, ValueGroup>;

}  // namespace SOTG