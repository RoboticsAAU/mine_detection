#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>

ros::Subscriber sub_pose;

void poseCallback(const nav_msgs::Odometry::ConstPtr &pose_message){
    ROS_INFO_STREAM("Angular: " << pose_message->twist.twist.angular.z << ", Linear: " << pose_message->twist.twist.linear.x );
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gazebo_odom");
    ros::NodeHandle n;

    sub_pose = n.subscribe("/odom", 10, &poseCallback);

    while(ros::ok()){
        ros::spinOnce();
    }
}
