#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <iostream>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber sub_pose;

struct Vector2D
{
    double x;
    double y;
};

bool IsClockwise(double angleActual, double angleDesired);
bool VectorInUpperQuadrants(Vector2D vector);
Vector2D rotateVectorByAngle(double angle, Vector2D vector);
Vector2D vectorByAngle(double angle);

//method to move the robot straight.

void move(double speed, double distance, bool isForward);
double getTheta(double angle);
void rotate(double angular_speed, double angle);
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);
double euclidean_distance(double x1, double y1, double x2, double y2);
double linear_velocity(turtlesim::Pose goal);
double angular_velocity(turtlesim::Pose goal);
double getAngle(turtlesim::Pose goal);
void move2goal(turtlesim::Pose goal);
void setAllVelocityZero();

const double distance_tolerance = 0.01;

turtlesim::Pose cur_pose;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n;

    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    sub_pose = n.subscribe("/turtle1/pose", 1000, &poseCallback);

    ros::Rate loop_rate(10);

    turtlesim::Pose goal_pose;

    //goal_pose.x = 1;
    //goal_pose.y = 3;
    //move2goal(goal_pose);


    while(cur_pose.x == 0){
        std::cout << cur_pose.x << ":" << cur_pose.y << endl;
        ros::spinOnce();
    };
    std::cout << cur_pose.x << ":" << cur_pose.y << endl;

    for (int i = 1; i < 11; i++)
    {

        if (i % 2 == 1)
        {
            for (int j = 1; j < 11; j++)
            {
                goal_pose.x = i;
                goal_pose.y = j;
                rotate(2.0, getTheta(getAngle(goal_pose)));
                std::cout << "Going to: " << goal_pose.x << " , " << goal_pose.y << endl;
                move2goal(goal_pose);
            }
        }
        else
        {
            for (int j = 10; j > 0; j--)
            {
                std::cout << "Going to: " << i << " , " << j << endl;
                goal_pose.x = i;
                goal_pose.y = j;
                std::cout << getTheta(getAngle(goal_pose)) << endl;
                rotate(2.0, getTheta(getAngle(goal_pose)));
                move2goal(goal_pose);
            }
        }
    }

    //move2goal(goal_pose);
    //setDesiredOrientation(M_PI);
    //move2goal(goal_pose);
    //rotate(2.0, M_PI_2);

    //cout << "done";
    //goal_pose.theta = 2 * M_PI;

    // while(ros::ok()){
    //     cout << cur_pose.theta << " , " << getTheta() << endl;
    //     ros::spinOnce();
    // }

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

    setAllVelocityZero();

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(5);
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

double getTheta(double angle)
{
    double theta = angle < 0 ? angle + 2 * M_PI : angle;
    //cout << "Got theta: " << theta << endl;
    return theta;
}

void rotate(double angular_velocity, double desired_angle)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    //setAllVelocityZero();

    ros::spinOnce();
    


    // Rotates either clockwise (if=true) or counterclockwise (if=false) depending on which is shortest.
    if (IsClockwise(getTheta(cur_pose.theta), desired_angle))
    {
        vel_msg.angular.z = -fabs(angular_velocity);
    }
    else
    {
        vel_msg.angular.z = fabs(angular_velocity);
    }
    
    ros::Rate loop_rate(1000);
    // Rotates until turtle has rotated to desired angle (within 0.05 radians).
    do
    {
        //cout << cur_pose.x << ":" << cur_pose.y << endl;
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    } while (fabs(desired_angle - getTheta(cur_pose.theta)) > 0.02 && ros::ok());
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}

#pragma region WIP
bool IsClockwise(double angleActual, double angleDesired)
{
    Vector2D vectorActual = vectorByAngle(angleActual);
    Vector2D vectorDesired = vectorByAngle(angleDesired);

    Vector2D rotatedVectorActual = rotateVectorByAngle(angleActual, vectorActual);
    Vector2D rotatedVectorDesired = rotateVectorByAngle(angleActual, vectorDesired);

    return !VectorInUpperQuadrants(rotatedVectorDesired);
}

bool VectorInUpperQuadrants(Vector2D vector)
{
    return vector.y >= 0;
}

Vector2D rotateVectorByAngle(double angle, Vector2D vector)
{
    Vector2D rotatedVector;
    rotatedVector.x = vector.x * cos(2 * M_PI - angle) + vector.y * (-sin(2 * M_PI - angle));
    rotatedVector.y = vector.x * sin(2 * M_PI - angle) + vector.y * cos(2 * M_PI - angle);
    return rotatedVector;
}

Vector2D vectorByAngle(double angle)
{
    Vector2D vector;
    vector.x = cos(angle);
    vector.y = sin(angle);
    return vector;
}

#pragma endregion

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
    double kv = 1.5;

    double distance = euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y);

    //cout << "Distance: " << distance << endl;

    return kv * distance;
}

double angular_velocity(turtlesim::Pose goal)
{
    double ka = 4;

    return ka * (getTheta(getAngle(goal)) - getTheta(cur_pose.theta));
}

double getAngle(turtlesim::Pose goal)
{
    return atan2(goal.y - cur_pose.y, goal.x - cur_pose.x);
}

void move2goal(turtlesim::Pose goal)
{

    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate = (100);

    while (euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y) > distance_tolerance && ros::ok())
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
        loop_rate.sleep();
        ros::spinOnce();
    }
    //setAllVelocityZero();
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}

void setAllVelocityZero()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
}
