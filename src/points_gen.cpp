#include "points_gen.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <visualization_msgs/Marker.h>

using namespace Points_gen;

std::vector<Point> points_List::gen_Point_list()
{
    //create a new vector of points.
    std::vector<Point> vec;

    //m is length
    //n is width
    double m = 5.0;
    double n = 5.0;
    //Distance between points
    double point_distance = 0.1;
    //robot radius in meters
    double robot_radius = 0.175;
    //count used to generate the points in the correct order.
    int count = 0;

    //iterate through the length of the field.
    for (double i = robot_radius; i < m; i += 2 * robot_radius)
    {
        //check if the iteration is even.
        if (count % 2 == 0)
        {
            //iterate through the field in correct assending order.
            for (double j = robot_radius; j < n; j += point_distance)
            {
                //create a point.
                Point p = {i, j};
                //if the first point in iteration, set stop to true.
                if (j == robot_radius)
                {
                    p.stop = true;
                }
                //append the point to the vector.
                vec.push_back(p);
            }
        }
        else
        {
            //if count is uneven, itterate through in dessending order.
            for (double j = n - robot_radius; j > 0; j -= point_distance)
            {
                //create a point.
                Point p = {i, j};
                //if the first point in iteration, set stop to true.
                if (j == n - robot_radius)
                {
                    p.stop = true;
                }
                //append the point to the vector
                vec.push_back(p);
            }
        }
        //set the last point in iteration to stop.
        vec.at(vec.size() - 1).stop = true;
        //increment count.
        count++;
    }
    ROS_INFO("Created points succesfully");
    //return vector.
    return vec;
}

void points_List::rvizPoints(ros::Publisher point_pub, std::vector<Point> point_list)
{
    visualization_msgs::Marker points, line_strip;

    //Initializing the points and line_strip object data.
    //The default marker message members are 0.
    //frame id has to be the same as the robots position topic.
    points.header.frame_id = line_strip.header.frame_id = "/odom";
    points.ns = line_strip.ns = "Path namespace";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;

    //w must be a non-zero value to be displayed.
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    //timestamp
    points.header.stamp = line_strip.header.stamp = ros::Time::now();

    //assign id to the points, to avoid
    //other broadcasters colliding with points.
    points.id = 0;
    line_strip.id = 1;

    //specify the type of markermessage, using an enum.
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    //scale the points, using x and y, specifying the width and height.
    points.scale.x = 0.02;
    points.scale.y = 0.02;

    //set thickness of the lines.
    line_strip.scale.x = 0.02;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //set lifetime of the markers
    points.lifetime = ros::Duration();
    line_strip.lifetime = ros::Duration();

    //iterate through points_list
    for (size_t i = 0; i < point_list.size(); ++i)
    {
        //create objects
        std_msgs::ColorRGBA color;
        geometry_msgs::Point point;

        //set the colors of the points using RGBA, based on if the point is a stop point.
        if (point_list[i].stop)
        {
            //RED
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 1.0;
        }
        else
        {
            //GREEN
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 1.0;
        }

        //add the pointlist elements fields to point.
        point.x = point_list[i].x;
        point.y = point_list[i].y;

        //append color and, point into the color, and point vector.
        //there must be the same amount of elements in each list, else there is a fail.
        points.colors.push_back(color);
        points.points.push_back(point);

        //append the linestrip.
        line_strip.points.push_back(point);

        //publish
        point_pub.publish(points);
        point_pub.publish(line_strip);
    }
    //process callbacks.
    ros::spinOnce();
    ROS_INFO("Done publishing rviz points.");
}
