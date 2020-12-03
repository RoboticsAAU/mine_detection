/*#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <turtlesim/Pose.h>
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
ros::Subscriber point_sub;

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "paper_detector");
    ros::NodeHandle n;
    marker_pub= n.advertise<geometry_msgs::Point>("/paper/pose", 10);
    point_sub= n.subscribe("/turtle1/pose", 10,&poseCallback);

    while (ros::ok())
  {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "point_pose";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::point;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point_sub.x;
        marker.pose.position.y = point_sub.y;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.g = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
  }
}*/
/*#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "pointsToRviz");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
  ros::Subscriber point_sub = n.subscribe("/paper/pose", 10,);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "paper_pose";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::POINTS;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
  

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    r.sleep();
  }
}
*/