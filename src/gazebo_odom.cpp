#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>

ros::Subscriber sub_pose;

#pragma region Quaterion To Euler Angles conversion
struct Quaternion
{
    double x, y, z, w;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

//convert quarternion into eulerangles.
EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    //roll (x axis rotation)
    double sinr_cosp = 2 * (q.w * q.x - q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    //pitch (y axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        //copysign returns the magnitude of M_PI/2 with the sign of sinp
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
#pragma endregion

void poseCallback(const nav_msgs::Odometry::ConstPtr &pose_message)
{
    //ROS_INFO_STREAM("Angular: " << pose_message->twist.twist.angular.z << ", Linear: " << pose_message->twist.twist.linear.x );

    Quaternion q;

    //assign values of pose message to quaternion.
    q.x = pose_message->pose.pose.orientation.x;
    q.y = pose_message->pose.pose.orientation.y;
    q.z = pose_message->pose.pose.orientation.z;
    q.w = pose_message->pose.pose.orientation.w;

    double yaw = ToEulerAngles(q).yaw + M_PI;
    double pitch = ToEulerAngles(q).pitch;
    double roll = ToEulerAngles(q).roll;

    std::cout << "yaw: " << yaw << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "roll: " << roll << std::endl
              << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gazebo_odom");
    ros::NodeHandle n;

    sub_pose = n.subscribe("/odom", 10, &poseCallback);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}
