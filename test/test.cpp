#include <iostream>

#include <sotg/sotg.hpp>

int main()
{
    SOTG::SymbolGroupMap symbol_map;
    symbol_map["pos"] = {"x", "y", "z"};
    symbol_map["rot"] = {"a", "b", "c"};

    // TrajectoryGenerator tg(symbol_map);
}