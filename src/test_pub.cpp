#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <turtlesim/Pose.h>

using namespace std;

turtlesim::Pose cur_pose;

ros::Subscriber sub_pose;

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
    cur_pose.x = pose_message->x;
    cur_pose.y = pose_message->y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    sub_pose = n.subscribe("/paper_pose", 10, &poseCallback);

    return 0;
}
