#include <iostream>

#include "sotg/sotg.hpp"

int main()
{
    // SOTG::SymbolGroupMap symbol_map;
    // symbol_map["pos"] = {"x", "y", "z"};
    // symbol_map["rot"] = {"a", "b", "c"};

    // SOTG::Point2 p1(symbol_map, "p1");
    // p1["pos"] = {0.0, 0.0, 0.0};
    // p1["rot"] = {0.0, 0.0, 0.0};

    // p1["pos"].setConstraints(1.0, 1.0);
    // p1["rot"].setConstraints(1.0, 1.0);

    // SOTG::Point2 p2(symbol_map, "p2");
    // p2["pos"] = {1.0, 0.0, 0.0};
    // p2["rot"] = {0.0, 0.0, 0.0};

    // p2["pos"].setConstraints(1.0, 1.0);
    // p2["rot"].setConstraints(1.0, 1.0);

    SOTG::SymbolGroupMap symbol_map;
    symbol_map["pos"] = {"x", "y"};
    symbol_map["rot"] = {"a"};

    SOTG::Point2 p1(symbol_map, "p1");
    p1["pos"] = {0.0, 0.0};
    p1["rot"] = {0.0};

    p1["pos"].setConstraints(1.0, 1.0);
    p1["rot"].setConstraints(1.0, 1.0);

    SOTG::Point2 p2(symbol_map, "p2");
    p2["pos"] = {2.0, 1.0};
    p2["rot"] = {1.0};

    p2["pos"].setConstraints(1.0, 1.0);
    p2["rot"].setConstraints(1.0, 4.0);


    // std::cout << p1 << std::endl;

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
    my_path.addPoint(p1);

    SOTG::TrajectoryGenerator2 tg(symbol_map);

    tg.resetPath(my_path);

    double duration = tg.getDuration();

    std::cout << "Duration: " << duration << "s" << std::endl;

    SOTG::Result result(symbol_map);

    for (size_t i = 0; i < 100; ++i)
    {
        double tick = i/100.0 * duration;
        tg.calcPositionAndVelocity(tick, result);
        std::cout << " --- " << tick << "s --- " << std::endl;
        std::cout << result.getLocation() << std::endl;
        std::cout << result.getVelocity() << std::endl;
        std::cout << " --- " << std::endl;

    }


}