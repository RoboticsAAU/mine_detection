#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>

struct Point {
    int x;
    int y;
};


std::vector<Point> environmentMap;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
void rvizPoints(ros::Publisher point_pub, std::vector<Point> point_list);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_gen");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &mapCallback);
    ros::Publisher point_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    while(ros::ok()){
        ros::spinOnce();
        rvizPoints(point_pub,environmentMap);
    }

    return 0;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg){
    environmentMap.clear();
    environmentMap.shrink_to_fit();
    for (size_t i = 0; i < map_msg->data.; i++)
    {
        /* code */
    }
    
    {
        int i = 0;
        std::cout << cell << std::endl;
        
            Point p;

            p.x = map_msg->info.origin.position.x + ( - map_msg->info.width/2) * map_msg->info.resolution;
            p.y = map_msg->info.origin.position.y + ( - map_msg->info.height/2) * map_msg->info.resolution;

            
            ROS_INFO("Calculating point");
            std::cout << p.x << " , " << p.y << std::endl;
            environmentMap.push_back(p);
        
        //transform into real evironment data. 
    }
    
}

void rvizPoints(ros::Publisher point_pub, std::vector<Point> point_list)
{
    visualization_msgs::Marker points, line_strip;

    //Initializing the points and line_strip object data.
    //The default marker message members are 0.
    //frame id has to be the same as the robots position topic.
    points.header.frame_id = line_strip.header.frame_id = "/map";
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
        //GREEN
        color.r = 0.0f;
        color.g = 1.0f;
        color.b = 0.0f;
        color.a = 1.0;
        

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
    //ROS_INFO("Done publishing rviz points.");
}