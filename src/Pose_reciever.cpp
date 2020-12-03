#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <turtlesim/Pose.h>
#include "geometry_msgs/Point.h"

ros::Publisher Marker_pub;
ros::Subscriber point_sub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "paper_detector");
    ros::NodeHandle n;
    Marker_pub= n.advertise<geometry_msgs::Point>("/paper/pose", 10);
    point_sub= n.subscribe("/turtle1/pose", 10);