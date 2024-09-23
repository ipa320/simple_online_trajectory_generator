#pragma once

#include "sotg/symbol_group.hpp"
#include "sotg/value_group.hpp"

namespace SOTG {

// Contains the limiting factors for a section
class SectionConstraint2 {
private:
    double acceleration_magnitude_linear_;
    double acceleration_magnitude_angular_;

    double velocity_magnitude_linear_;
    double velocity_magnitude_angular_;

    SymbolGroupMap& symbol_map_;
    ValueGroupMap value_map_;

public:
    SectionConstraint2(SymbolGroupMap& symbol_map) : symbol_map_(symbol_map) {
        for (auto& [key, symbols] : symbol_map) {
            value_map_.insert({key, ValueGroup(symbols)});
        }
    }

    double getAccelerationMagnitudeLinear() const { return acceleration_magnitude_linear_; }
    double getAccelerationMagnitudeAngular() const { return acceleration_magnitude_angular_; }
    double getVelocityMagnitudeLinear() const { return velocity_magnitude_linear_; }
    double getVelocityMagnitudeAngular() const { return velocity_magnitude_angular_; }
};

}  // namespace SOTG