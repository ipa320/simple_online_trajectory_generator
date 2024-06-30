#pragma once

#include <initializer_list>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

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
    std::map<std::string, double> values_;
    const SymbolGroup& symbols_;

public:
    ValueGroup(const SymbolGroup& symbol_group)
        : symbols_(symbol_group)
    {
    }

    void operator=(std::initializer_list<double> list)
    {
        for (size_t i = 0; i < list.size(); i++) {
            values_[symbols_[i]] = list.begin()[i];
        }
    }
    double& operator[](std::string key) { return values_[key]; }

    bool is_quaternion = false;
};

using ValueGroupMap = std::unordered_map<std::string, ValueGroup>;

}  // namespace SOTG