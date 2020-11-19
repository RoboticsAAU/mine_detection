#include "move.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <iostream>


using namespace N;

void Move::move(double speed, double distance, bool isForward, ros::Publisher* ptr)
{
    ros::Publisher vel_pub = *ptr;

    geometry_msgs::Twist vel_msg;


    if (isForward)
        vel_msg.linear.x = abs(speed);
    else
    {
        vel_msg.linear.x = -abs(speed);
    }

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(100);
    do
    {
        vel_pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();

    } while (current_distance < distance);
    vel_msg.linear.x = 0;
    vel_pub.publish(vel_msg);

    std::cout << "Done" << std::endl;
    //distance = speed * time
}

void Move::print(int i)
{
    std::cout << i << std::endl;
}