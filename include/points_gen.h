#pragma once
#include <vector>
#include "ros/ros.h"

namespace Points_gen
{
    struct Point
    {
        double x;
        double y;
        bool stop;
    };

    class points_List
    {
    public:
        std::vector<Point> gen_Point_list();
        void rvizPoints(ros::Publisher point_pub, std::vector<Point> point_list);
    };

} // namespace Points_gen