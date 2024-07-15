#include <iostream>

#include "sotg/sotg.hpp"

int main()
{
    SOTG::SymbolGroupMap symbol_map;
    symbol_map["pos"] = {"x", "y", "z"};
    symbol_map["rot"] = {"a", "b", "c"};

    SOTG::Point2 p1(symbol_map, "p1");
    p1["pos"] = {0.0, 0.0, 0.0};
    p1["rot"] = {0.0, 0.0, 0.0};

    SOTG::Point2 p2(symbol_map, "p2");
    p2["pos"] = {1.0, 0.0, 0.0};
    p2["rot"] = {0.0, 0.0, 0.0};

    std::cout << p1 << std::endl;

    // SOTG::SymbolGroupMap symb_map2;
    // symb_map2["A1"] = {"a"};
    // symb_map2["A2"] = {"a"};
    // symb_map2["A3"] = {"a"};
    // symb_map2["A4"] = {"a"};
    // symb_map2["A5"] = {"a"};
    // symb_map2["A6"] = {"a"};

    // SOTG::Point2 p2(symb_map2, "p2");
    // p2["A3"] = {0.25};
    // p2["A5"] = {0.0};

    // std::cout << p2 << std::endl;

    SOTG::Path2 my_path;
    my_path.addPoint(p1);
    my_path.addPoint(p2);

    SOTG::TrajectoryGenerator2 tg(symbol_map);

    tg.resetPath(my_path);
}