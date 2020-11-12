#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <iostream>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber sub_pose;

//method to move the robot straight.

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);
double euclidean_distance(double x1, double y1, double x2, double y2);
double linear_velocity(turtlesim::Pose goal);
double angular_velocity(turtlesim::Pose goal);
void move2goal(turtlesim::Pose goal);
void setDesiredOrientation(double desired_angle_radians);
double degrees2radians(double angle_in_degrees);

const double distance_tolerance = 0.1;

const double pi = M_PI;

//double cur_x, cur_y, cur_theta;

turtlesim::Pose cur_pose;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n;

    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    sub_pose = n.subscribe("/turtle1/pose", 10, &poseCallback);

    ros::Rate loop_rate(10);

    // ROS_INFO("\n\n\n ***************START TESTING ************\n");
    // cout << "enter speed: ";
    // cin >> speed;
    // cout << "Enter distance: ";
    // cin >> distance;
    // cout << "Forward?: ";
    // cin >> isForward;

    // move(speed, distance, isForward);

    // cout << "Enter angular speed: ";
    // cin >> angular_speed;
    // cout << "Enter angle: ";
    // cin >> angle;
    // cout << "Clockwise?: ";
    // cin >> clockwise;

    // rotate(angular_speed,angle,clockwise);

    turtlesim::Pose goal_pose;
    // std::cout << "Set your x goal: ";
    // std::cin >> goal_pose.x;
    // std::cout << "Set your y goal: ";
    // std::cin >> goal_pose.y;
    // std::cout << "Set your theta goal: ";
    // std::cin >> goal_pose.theta;
    goal_pose.x = 1;
    goal_pose.y = 3;
    //rotate(1,M_PI,0);
    //setDesiredOrientation(M_PI + M_PI_2);
    //degrees2radians(90);
    // for (int i = 1; i < 11; i++)
    // {
    //     if(i%2 == 1){
    //         for (int j = 1; j < 11; j++)
    //         {
    //             goal_pose.x = i;
    //             goal_pose.y = j;
    //             move2goal(goal_pose);
    //         }
    //     }
    //     else {
    //         for (int j = 10; j > 0; j--)
    //         {
    //             goal_pose.x = i;
    //             goal_pose.y = j;
    //             move2goal(goal_pose);
    //         }

    //     }

    // }

    move2goal(goal_pose);
    setDesiredOrientation(M_PI_2);

    //cout << "done";
    //goal_pose.theta = 2 * M_PI;

    return 0;
}

/**
 * makes the robot move forward with a certain linear velocity for a
 * certain distance in a forward or backward straight direction.
 * */

void move(double speed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;

    if (isForward)
        vel_msg.linear.x = abs(speed);
    else
    {
        vel_msg.linear.x = -abs(speed);
    }
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

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

    //distance = speed * time
}

//positive angular speed rotation is counteclockwise.
void rotate(double angular_speed, double angle, bool clockwise)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    if (clockwise)
    {
        vel_msg.angular.z = -abs(angular_speed);
    }
    else
    {
        vel_msg.angular.z = abs(angular_speed);
    }

    double current_angle = cur_pose.theta;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);
    do
    {
        vel_pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
    } while (current_angle <= angle);
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{

    cur_pose.x = pose_message->x;
    cur_pose.y = pose_message->y;
    cur_pose.theta = pose_message->theta;
    //ROS_INFO_STREAM("position=(" << cur_pose.x << "," << cur_pose.y << ")" << " angle= " << cur_pose.theta );

    //std::cout << "x: " << cur_pose.x << std::endl << "y: " << cur_pose.y << std::endl << "theta: " << cur_pose.theta << std::endl;

    // cur_x = pose_message.x;
    // cur_y = pose_message.y;
    // cur_theta = pose_message.theta;
}

double euclidean_distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

double linear_velocity(turtlesim::Pose goal)
{
    double kv = 1;

    double distance = euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y);

    //cout << "Distance: " << distance << endl;

    return kv * distance;
}

double angular_velocity(turtlesim::Pose goal)
{
    double ka = 4;

    return ka * (atan2(goal.y - cur_pose.y, goal.x - cur_pose.x) - cur_pose.theta);
}

void move2goal(turtlesim::Pose goal)
{

    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate = (10);

    while (euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y) > distance_tolerance)
    {
        // std::cout << "x: " << cur_pose.x << std::endl << "y: " << cur_pose.y << std::endl << "theta: " << cur_pose.theta << std::endl;

        vel_msg.linear.x = linear_velocity(goal);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = angular_velocity(goal);

        vel_pub.publish(vel_msg);

        //cout << "still looping" << endl;
        if (euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y) > distance_tolerance)
        {
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}

void setDesiredOrientation(double desired_angle_radians)
{
    double relative_angle_radians = desired_angle_radians - cur_pose.theta;
    //if we want to turn at a perticular orientation, we subtract the current orientation from it
    bool clockwise = ((relative_angle_radians < 0) ? true : false);
    cout << desired_angle_radians << "," << cur_pose.theta << "," << relative_angle_radians << "," << clockwise << endl;
    rotate(2, abs(relative_angle_radians), clockwise);
}

double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * M_PI / 180.0;
}