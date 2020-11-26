#pragma once
#include <vector>

namespace Obstacle_avoidance{
    struct Obstacle_Point{
        double x;
        double y;
    };
    class Obstacle{
        std::vector<Obstacle_Point> peripheral_points();
    };
}