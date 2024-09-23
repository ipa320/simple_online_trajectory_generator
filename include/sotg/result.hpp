#pragma once

#include "sotg/symbol_group.hpp"
#include "sotg/value_group.hpp"
#include "sotg/point2.hpp"

namespace SOTG {

class Result
{
private:
    SymbolGroupMap& symbol_map_;
    Point2 location, velocity;

public:
    Result(SymbolGroupMap& symbol_map) : symbol_map_(symbol_map), location(symbol_map, "location"), velocity(symbol_map, "velocity") {

    }

    Point2& getLocation() { return location; }
    Point2& getVelocity() { return velocity; }
};

} // Namespace SOTG