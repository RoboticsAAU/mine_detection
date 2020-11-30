#include "points_gen.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <visualization_msgs/Marker.h>

using namespace Points_gen;

std::vector<Point> points_List::gen_Point_list()
{
    std::vector<Point> vec;

    //std::cout << vec.capacity() << std::endl;
    double m = 5.0;
    double n = 5.0;
    double robot_radius = 0.175;
    int count = 0;

    for (double i = robot_radius; i < m; i += 2 * robot_radius)
    {
        //std::cout << count % 2 << std::endl;
        //std::cout << fmod(i,robot_radius) << std::endl;
        if (count % 2 == 0)
        {

            for (double j = robot_radius; j < n; j += 2 * robot_radius)
            {
                Point p = {i, j};
                std::cout << p.x << "," << p.y << std::endl;
                //std::cin.get();
                //p.stop = false;
                if (j == robot_radius)
                {
                    p.stop = true;
                }
                vec.push_back(p);
            }
        }
        else
        {
            for (double j = n - robot_radius; j > 0; j -= 2 * robot_radius)
            {
                Point p = {i, j};
                std::cout << p.x << "," << p.y << std::endl;
                //std::cin.get();
                if (j == n - robot_radius)
                {
                    p.stop = true;
                }
                vec.push_back(p);
            }
        }
        vec.at(vec.size() - 1).stop = true;
        count++;
    }

    std::cin.get();
    /*
    for(double i = robot_radius; i < m-robot_radius; i + 2*robot_radius){
        std::cout << i << std::endl;
        for(double j = robot_radius; j < n - robot_radius; j + 2*robot_radius){
            Point p = {i,j};
            std::cout << p.x << "," << p.y << std::endl;
            std::cin.get();
            vec.push_back(p);
        }
    }
    */

    return vec;
}

void points_List::rvizPoints(ros::Publisher point_pub, std::vector<Point> point_list)
{
    ros::Rate loop_rate(10);
    while (point_pub.getNumSubscribers() < 1)
    {
        visualization_msgs::Marker points;

        //Initializing the points object data.
        //The default marker message members are 0.
        points.header.frame_id = "/path_frame";
        points.ns = "points namespace";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.header.stamp = ros::Time::now();
        //assign id to the points, to avoid
        //other broadcasters colliding with points.
        points.id = 0;

        //specify the type of markermessage, using an enum.
        points.type = visualization_msgs::Marker::POINTS;

        //scale the points, using x and y, specifying the width and height.
        points.scale.x = 0.02;
        points.scale.y = 0.02;
        //points.scale.z = 1;

        //set the colors of the points using RGBA, to green.
        points.color.g = 200.0f;
        points.color.b = 1.0f;
        points.color.r = 1.0f;
        points.color.a = 0.9;
        //points.lifetime = ros::Duration();

        for (Point p : point_list)
        {
            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;
            points.points.push_back(point);
            point_pub.publish(points);
            loop_rate.sleep();
        }

        std::cout << "publishing point" << std::endl;
        ros::spinOnce();
    }
}
