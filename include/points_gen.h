#pragma once
#include <vector>

namespace Points_gen{
    struct Point
    {
        double x;
        double y;
    };

    class points_List{
    public:
        std::vector<Point>gen_Point_list(Point* p);
        
    };

    
  
}