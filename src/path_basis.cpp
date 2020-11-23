#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <iostream>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

#include <move.h>
#include <points_gen.h>

using namespace std;
using namespace N;
using namespace Points_gen;

#define Log(name, x) std::cout << name << ": " << x << std::endl;

ros::Publisher reset_pub;
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


double getTheta(double angle);
void rotate(turtlesim::Pose goal);
void poseCallback(const nav_msgs::Odometry::ConstPtr &pose_message);
double euclidean_distance(double x1, double y1, double x2, double y2);
double linear_velocity(turtlesim::Pose goal);
double angular_velocity(turtlesim::Pose goal);
double getAngle(turtlesim::Pose goal);
void move2goal(turtlesim::Pose goal);

const double distance_tolerance = 0.05;

turtlesim::Pose cur_pose;

#pragma region Quaternion To Euler Angles conversion
struct Quaternion
{
    double x, y, z, w;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles angles;

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
    //calculate yaw, and make the yaw angle be 0 < yaw < 2pi.
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
#pragma endregion

/*
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{

    cur_pose.x = pose_message->x;
    cur_pose.y = pose_message->y;
    cur_pose.theta = pose_message->theta;
    //ROS_INFO_STREAM("position=(" << cur_pose.x << "," << cur_pose.y << ")" << " angle= " << cur_pose.theta );

    //std::cout << "x: " << cur_pose.x << std::endl << "y: " << cur_pose.y << std::endl << "theta: " << cur_pose.theta << std::endl;
}
*/

void poseCallback(const nav_msgs::Odometry::ConstPtr &pose_message)
{
    //ROS_INFO_STREAM("Angular: " << pose_message->twist.twist.angular.z << ", Linear: " << pose_message->twist.twist.linear.x );

    // Get the x,y position.
    cur_pose.x = pose_message->pose.pose.position.x;
    cur_pose.y = pose_message->pose.pose.position.y;

    // Quaternion object q.
    Quaternion q;

    // Assign values of pose message to quaternion.
    q.x = pose_message->pose.pose.orientation.x;
    q.y = pose_message->pose.pose.orientation.y;
    q.z = pose_message->pose.pose.orientation.z;
    q.w = pose_message->pose.pose.orientation.w;

    // Retrieve Euler angles from quaternion pose message.
    angles = ToEulerAngles(q);

    cur_pose.theta = angles.yaw;

    //std::cout << "angle: " << angles.yaw << " x: " << cur_pose.x << " y: " << cur_pose.y << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mine_detection_path_planning");
    ros::NodeHandle n;

    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    reset_pub = n.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 10);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
    sub_pose = n.subscribe("/odom", 1000, &poseCallback);

    std::cout << "Resetting odometry..." << std::endl;
    while (reset_pub.getNumSubscribers() == 0)
    {
        ros::spinOnce();
    }
    
    std_msgs::Empty e;
    reset_pub.publish(e);
    std::cout << "Done" << std::endl;

    ros::Rate loop_rate(10);

    turtlesim::Pose goal_pose;

    std::cin.get();
    
    //create instance of the Move class.
    //Move move_instance;
   
    //create a pointer to the publisher, which holds the memory address of the pointer.
    //ros::Publisher* vel_pub_ptr = &vel_pub;
    
    //call the move function, and pass the pointer in as an argument. 
    //move_instance.move(2.0,2.0,true,vel_pub_ptr);

    points_List points_instance;

    Point p = {2,4};

    std::vector<Points_gen::Point> vec;
    vec = points_instance.gen_Point_list(&p);

    std::cout << vec.at(0).x << "," << vec.at(0).y << std::endl;
    std::cin.get();


    for(Point p : vec){
        std::cout << "moving..." << std::endl;
        std::cout << "Going to: " << goal_pose.x << " , " << goal_pose.y << endl;
        goal_pose.x = p.x;
        goal_pose.y = p.y;
        //std::cin.get();
        rotate(goal_pose);
        move2goal(goal_pose);
    }
    std::cin.get();

    // The while loop fixes a bug where the turtle's coordinates are wrong when it spawns, by waiting for the turle's position to be updated.
    // The turtle thinks it spawns at (0 ; 0), but it actually spawns at around (5,5 ; 5,5))
    while (cur_pose.x == 0)
    {
        ros::spinOnce();
    }

    // The for loop as a whole is what makes the turtle move to the correct places in the correct order, thus making the boustrophedon decomposition.
    // The outmost for loop is the overall amount of straight lines laterally (It does 10 lines).
    for (int i = 1; i < 11; i++)
    {
        // When the x-coordinate is odd, the turtle moves upwards (increase in y-values).
        if (i % 2 == 1)
        {
            for (int j = 1; j < 11; j++)
            {
                std::cout << "Going to: " << goal_pose.x << " , " << goal_pose.y << endl;
                goal_pose.x = i;
                goal_pose.y = j;
                rotate(goal_pose);
                move2goal(goal_pose);
            }
        }
        // When the x-coordinate is even, the turtle moves downwards (decrease in y-values).
        else
        {
            for (int j = 10; j > 0; j--)
            {
                std::cout << "Going to: " << i << " , " << j << endl;
                goal_pose.x = i;
                goal_pose.y = j;
                rotate(goal_pose);
                move2goal(goal_pose);
            }
        }
        // The turtle moves to the side when a new iteration happens (since the x-value increases by 1).
    }

    return 0;
}

/**
 * makes the robot move forward with a certain linear velocity for a
 * certain distance in a forward or backward straight direction.
 * */



double getTheta(double angle)
{
    //If theta is negative it is converted to the corresponding positive angle (Theta becomes negative when the turtle rotates clockwise).
    double theta = angle < 0 ? angle + 2 * M_PI : angle;
    //cout << "Got theta: " << theta << endl;
    return theta;
}

void rotate(turtlesim::Pose goal)
{
    geometry_msgs::Twist vel_msg;

    double desired_angle = getTheta(getAngle(goal));

    // Sets all the velocities equal to zero.
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    ros::spinOnce();

    // Rotates either clockwise (if=true) or counterclockwise (if=false) depending on which is shortest.
    

    ros::Rate loop_rate(1000);
    // Rotates until turtle has rotated to desired angle (within 0.02 radians).
    do
    {
        if (IsClockwise(getTheta(cur_pose.theta), desired_angle))
    {
        vel_msg.angular.z = -fabs(angular_velocity(goal));
    }
    else
    {
        vel_msg.angular.z = fabs(angular_velocity(goal));
    }

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    } while (fabs(desired_angle - getTheta(cur_pose.theta)) > 0.05 && ros::ok());

    // Stops the turtle from rotating.
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}

#pragma region WIP
// The function determines if the shortest rotation between the actual angle and the desired angle is clockwise or counterclockwise.
bool IsClockwise(double angleActual, double angleDesired)
{
    Vector2D vectorActual = vectorByAngle(angleActual);
    Vector2D vectorDesired = vectorByAngle(angleDesired);

    Vector2D rotatedVectorActual = rotateVectorByAngle(angleActual, vectorActual);
    Vector2D rotatedVectorDesired = rotateVectorByAngle(angleActual, vectorDesired);

    return !VectorInUpperQuadrants(rotatedVectorDesired);
}

// The function determines whether the rotated desired vector is in the upper quadrants (above the x-axis) or not.
bool VectorInUpperQuadrants(Vector2D vector)
{
    // If the vectors y-value is greater than zero it means that the vector is in the upper quadrants.
    return vector.y >= 0;
}

// The function rotates a vector around Origo and the x-axis by a given angle.
Vector2D rotateVectorByAngle(double angle, Vector2D vector)
{
    Vector2D rotatedVector;
    rotatedVector.x = vector.x * cos(2 * M_PI - angle) + vector.y * (-sin(2 * M_PI - angle));
    rotatedVector.y = vector.x * sin(2 * M_PI - angle) + vector.y * cos(2 * M_PI - angle);
    return rotatedVector;
}

// The function takes in an angle and makes it into a unit vector.
Vector2D vectorByAngle(double angle)
{
    Vector2D vector;
    vector.x = cos(angle);
    vector.y = sin(angle);
    return vector;
}
#pragma endregion

double euclidean_distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

double linear_velocity(turtlesim::Pose goal)
{
    double kv = 0.5;
    double max_linear_vel = 0.5;

    double linear_vel = kv * euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y);

    return fabs(linear_vel) > max_linear_vel ? copysign(max_linear_vel,linear_vel) : linear_vel;
}

double angular_velocity(turtlesim::Pose goal)
{
    double ka = 0.5;
    double max_angular_vel = 0.5;

    double angular_vel = ka * (getTheta(getAngle(goal)) - getTheta(cur_pose.theta));

    return fabs(angular_vel) > max_angular_vel ? copysign(max_angular_vel,angular_vel) : angular_vel;
}

// The function determines in which direction (meaning at what angle) it should move to get from the current position to the goal position.
double getAngle(turtlesim::Pose goal)
{
    return atan2(goal.y - cur_pose.y, goal.x - cur_pose.x);
}

// The function makes the turtle move to the given goal.
void move2goal(turtlesim::Pose goal)
{

    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate = (100);

    while (euclidean_distance(cur_pose.x, cur_pose.y, goal.x, goal.y) > distance_tolerance && ros::ok())
    {
        // std::cout << "x: " << cur_pose.x << std::endl << "y: " << cur_pose.y << std::endl << "theta: " << cur_pose.theta << std::endl;

        // Sets the linear velocity in the direction of the x-axis to a decreasing speed (look at linear_velocity function) depending on where the goal is.
        vel_msg.linear.x = linear_velocity(goal);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        // Sets the angular veloity around the z-axis to a speed (look at angular_velocity function) depending on the where the goal is.
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = angular_velocity(goal);

        vel_pub.publish(vel_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }

    // Sets the velocity (in all directions and rotations) to zero.
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}
