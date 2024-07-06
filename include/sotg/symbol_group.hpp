
#pragma once

#include <initializer_list>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace SOTG {

// The Blueprint for are SymbolGroupValue object.
// SymbolGroups are used in a SymbolMap to define the structure of the Poses that the trajectorie should cover.
// A SymbolGroup defines part of a Pose that share the same unit, e.g. Position in metres or Euler Angles in
// radians Exemple: a SymbolGroup could define two Positions x,y that are jointly limited in accelration and
// velocity, together with another SymbolGroup that defines a single Euler Angle that has its own angular
// acceleration and velocity limits, forms the basis for a 2D trajectory.

class SymbolGroup {
private:
    std::vector<std::string> symbols_;

public:
    SymbolGroup(std::initializer_list<std::string> list)
        : symbols_(list)
    {
    }

    SymbolGroup() { }

    std::vector<std::string>::iterator begin() { return symbols_.begin(); }
    std::vector<std::string>::iterator end() { return symbols_.end(); }
    size_t size() const { return symbols_.size(); }

    const std::vector<std::string>& getSymbols() const { return symbols_; }
    size_t getIndex(std::string key);

    std::string str() const;

    std::string operator[](size_t index) const { return symbols_[index]; }
    bool operator==(const SymbolGroup& symbol_group) const;
    bool operator!=(const SymbolGroup& symbol_group) const;

    bool is_quaternion = false;

    friend std::ostream& operator<<(std::ostream& out, const SymbolGroup& sg)
    {
        out << "[";
        for (auto& symbol : sg.symbols_) {
            out << " " << symbol << ",";
        }
        out << "\b";
        out << " ]";

        return out;
    }
};

using SymbolGroupMap = std::unordered_map<std::string, SymbolGroup>;

}  // namespace SOTG