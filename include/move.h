#ifndef MOVE_H
#define MOVE_H
#include "ros/ros.h"

namespace N
{
    class Move
    {
    public:
        void move(double speed, double distance, bool isForward,ros::Publisher* ptr);
        void print(int i);
    };
} // namespace N

#endif
