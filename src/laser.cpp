#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <vector>
#include <math.h>

//#include "obstacle.h"

struct Point
{
    double x;
    double y;
};

std::vector<Point> points;

ros::Subscriber laser_sub;
ros::Publisher laser_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{
    float angle;
    int count = 0;
    std::cout << "New array:" << std::endl;
    for (int i = 0; i < laser_msg->ranges.size(); i++)
    {
        angle = laser_msg->angle_min + (i * laser_msg->angle_increment);
        if (!std::isnan(laser_msg->ranges.at(i)))
        {
            if (0.5 < laser_msg->ranges[i] && laser_msg->ranges[i] < 1.5)
            {
                //count++;
                //std::cout << laser_msg->ranges.at(i) << std::endl;
                Point p;
                p.x = laser_msg->ranges[i] * cos(angle);
                p.y = laser_msg->ranges[i] * sin(angle);

                std::cout << p.x << " : " << p.y << std::endl;
                points.push_back(p);
            }
        }
    }
    //std::cout << laser_msg->ranges.size() << ", count is: " << count << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "laser_test");
    ros::NodeHandle n;

    laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &laserCallback);

    ros::Rate loop_rate(0.5);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
