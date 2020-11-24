#pragma once
#include <vector>

namespace Points_gen{
    struct Point
    {
        double x;
        double y;
        bool stop;
    };

    class points_List{
    public:
        std::vector<Point>gen_Point_list();
    };

    
  
}