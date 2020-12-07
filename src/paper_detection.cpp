#include "ros/ros.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <turtlesim/Pose.h>
#include "geometry_msgs/Point.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

using namespace std;

ros::Publisher point_pub;
ros::Subscriber sub_pose;
turtlesim::Pose cur_pose;
int iterationCount = 0;
class point
{
public:
     double x;
     double y;
};

double getTheta(double angle);
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);
double degreesToRadians(double angleDegrees);
point pixelsToMeters(point coordInPixels, double length);
point rotatePointByAngle(double angle, point coord);
point convertCoordinatesOfPoint(point Coord);
visualization_msgs::Marker pointToMark(point markcalc);

visualization_msgs::Marker marker_msg;

int main(int argc, char **argv)
{

     vector<int> lastRectSurface; //surface area of boundingbox last frame
     vector<bool> shouldPublish;
     int boundColour[] = {0, 0, 255};
     int contourColour[] = {0, 255, 0};
     int surflimit = 250;            //surflimit defines the lower boundary, where an object will be countoured and for which a bounding box will be made
     int upperLimitOfdecrease = 100; // this defines the upper limit for the change of the size of the bounding boxes
     int surfacedif;

     ros::init(argc, argv, "paper_detector");
     ros::NodeHandle n;
     point_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 100); //visualization_msgs::Marker /visualization_marker
     sub_pose = n.subscribe("/turtle1/pose", 10, &poseCallback);

     cv::VideoCapture cap(0); //Capture the video from webcam.
     //If the webcam cannot open, it is likely due to the iindex is wrong, thus it is trying to open a webcam that is not accessible through that index.

     if (!cap.isOpened()) //If not success, exit program.
     {
          std::cout << "Cannot open the web cam" << endl;
          return -1;
     }

     cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //Create a window called "Control".

     int iLowH = 170;
     int iHighH = 179;

     int iLowS = 170;
     int iHighS = 255;

     int iLowV = 83;
     int iHighV = 255;

     //Create trackbars in "Control" window.
     cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
     cv::createTrackbar("HighH", "Control", &iHighH, 179);

     cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
     cv::createTrackbar("HighS", "Control", &iHighS, 255);

     cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
     cv::createTrackbar("HighV", "Control", &iHighV, 255);

     while (ros::ok())
     {
          cv::Mat imgOriginal;

          bool bSuccess = cap.read(imgOriginal); //Read a new frame from video.

          if (!bSuccess) //If not success, break loop.
          {
               std::cout << "Cannot read a frame from video stream" << endl;
               break;
          }

          cv::Mat imgHSV;
          cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV.
          cv::Mat imgThresholded;
          cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image.

          //Morphological opening (removes small objects from the foreground).
          erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
          dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

          //Morphological closing (removes small holes from the foreground).
          dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
          erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

          vector<vector<cv::Point>> contours; // makes a 2D vector containing points

          findContours(imgThresholded, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

          vector<cv::Rect> boundbox(contours.size());
          vector<vector<cv::Point>> contours_poly(contours.size());

          for (size_t i = 0; i < contours.size(); i++)
          {

               approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
               boundbox[i] = boundingRect(contours_poly[i]);
          }

          for (size_t i = 0; i < contours.size(); i++)
          {
               drawContours(imgOriginal, contours_poly, -1, (contourColour[0], contourColour[1], contourColour[2]), 3);
               rectangle(imgOriginal, boundbox[i].tl(), boundbox[i].br(), (boundColour[0], boundColour[1], boundColour[2]), 2, 8, 0);
               //std::cout << boundbox[i].tl() << boundbox[i].br() <<  std::endl;
          }

          imshow("Thresholded Image", imgThresholded); //show the thresholded image
          imshow("Original", imgOriginal);             //show the original image

          vector<cv::Point> rectCenter(boundbox.size()); //current boundingbox center coordinates
          shouldPublish.resize(rectCenter.size(), true);
          //for (int j = 0; j =< rectCenter.size(); j++)
          //cout << " "
          vector<int> rectSurface(boundbox.size()); //surface area of boundingbox last frame
          lastRectSurface.resize(boundbox.size());  //surface area of boundingbox last frame

          for (size_t i = 0; i < boundbox.size(); i++)
          {
               rectSurface[i] = boundbox[i].width * boundbox[i].height;
               surfacedif = lastRectSurface[i] - rectSurface[i];
               if (surfacedif > surflimit) //if bounding rectangle is smaller than last frame save coordinated of bounding rectangle
               {
                    rectCenter[i] = {boundbox[i].x + (boundbox[i].width / 2), boundbox[i].y + (boundbox[i].height / 2)};

                    for (size_t i = 0; i < rectCenter.size(); i++)
                    {
                         point centerCoord;
                         centerCoord.x = double(rectCenter.at(i).x);
                         centerCoord.y = double(rectCenter.at(i).y);

                         if (centerCoord.x && centerCoord.y != 0 && shouldPublish[i] == true)
                         {
                              //point paperPoint = convertCoordinatesOfPoint(centerCoord);
                              // geometry_msgs::Point point_msg;
                              // point_msg.x = paperPoint.x;
                              // point_msg.y = paperPoint.y;
                              point_pub.publish(pointToMark(convertCoordinatesOfPoint(centerCoord)));
                              shouldPublish[i] = false;
                         }
                    }
               }
               //if(i > 2 && rectsurf[i-2]<rectsurf[i-1]>rectsurf[i])
               /*if (surfacedif < surflimit)
               {
               shouldPublish[i] = false;
               }*/
               lastRectSurface[i] = rectSurface[i];
          }

          if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
          {
               std::cout << "esc key is pressed by user" << endl;
               break;
          }
     }
     return 0;
}

//Everytime a message arrives, this function is called. It updates the robots coordinates and its orientation.
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
     cur_pose.x = pose_message->x;
     cur_pose.y = pose_message->y;
     cur_pose.theta = pose_message->theta;
}

double getTheta(double angle)
{
     //If theta is negative it is converted to the corresponding positive angle (Theta becomes negative when the turtle rotates clockwise).
     double theta = angle < 0 ? angle + 2 * M_PI : angle;
     //cout << "Got theta: " << theta << endl;
     return theta;
}

//Converts degrees to radians.
double degreesToRadians(double angleDegrees)
{
     return angleDegrees * M_PI / 180;
}

//Converts a pioint from pixels to meters, based on the ratio between side length and number of pixels.
point pixelsToMeters(point coordInPixels, double length)
{
     point coordInMeters;
     coordInMeters.x = coordInPixels.x * (length / 640);
     coordInMeters.y = coordInPixels.y * (length / 640);
     return coordInMeters;
}

//Rotates a vector by a given angle.
point rotatePointByAngle(double angle, point coord)
{
     point rotatedPoint;
     rotatedPoint.x = coord.x * cos(angle - M_PI_2) + coord.y * (-sin(angle - M_PI_2));
     rotatedPoint.y = coord.x * sin(angle - M_PI_2) + coord.y * cos(angle - M_PI_2);
     return rotatedPoint;
}

point convertCoordinatesOfPoint(point Coord)
{
     // Changable variables: Diagonal FOV of the camera, and the camera distance to the ground.
     double FOV = 64; //78
     double distFromGroundCam = 0.35;

     //The following determines the measurements (length and width) of the area that the camera projects.
     double halfFOV = degreesToRadians(FOV / 2);
     double B = M_PI - M_PI_2 - halfFOV;
     double a = (distFromGroundCam / sin(B)) * sin(halfFOV);
     double alpha = atan2(3, 4);

     double length = 2 * cos(alpha) * a;
     double width = 2 * sin(alpha) * a;
     //std::cout << "Dimensions: " << length << " ; " << width << "\n";

     //The coordinates of the found point in pixels.
     point coordInPixel; //Use the coordinates that will be published, the current values are for testing only.
     coordInPixel.x = Coord.x;
     coordInPixel.y = Coord.y;

     //Converts the point's coordinates from pixels to meters using the pixelsToMeters function.
     point coordInMeters = pixelsToMeters(coordInPixel, length);

     //Since the camera determines the coordinates of the point using the y-axis going downwards. The y-axis is reverted by adding a negative sign.
     coordInMeters.y = -coordInMeters.y;

     // The vector from the middle of the camera to the center of the robot. Measured as difference in x and y respectively and is changable.
     point camCenterToRobotCenter;
     camCenterToRobotCenter.x = 0;
     camCenterToRobotCenter.y = -0.21;

     //The vector of the area projected by the camera from Origo to the center of the area/camera.
     point camOrigoToCamCenter;
     camOrigoToCamCenter.x = 1.0 / 2.0 * length;
     camOrigoToCamCenter.y = -1.0 / 2.0 * width;
     //std::cout << "camOrigoToCamCenter: " << camOrigoToCamCenter.x << " ; " << camOrigoToCamCenter.y << "\n";

     //The vector from the projected area's Origo to the center of the robot.
     //This is done to shift the coodinate-system of the camera to a coodinate-system with Origo in the robot's centre.
     point camOrigoToRobot;
     camOrigoToRobot.x = camCenterToRobotCenter.x + camOrigoToCamCenter.x;
     camOrigoToRobot.y = camCenterToRobotCenter.y + camOrigoToCamCenter.y;
     //std::cout << "camOrigoToRobot: " << camOrigoToRobot.x << " ; " << camOrigoToRobot.y << "\n";

     //The vector of the found point from the robot centre (in meters).
     point coordInMetersToRobotOrigo;
     coordInMetersToRobotOrigo.x = coordInMeters.x - camOrigoToRobot.x;
     coordInMetersToRobotOrigo.y = coordInMeters.y - camOrigoToRobot.y;
     //std::cout << "coordInMetersToRobotOrigo: " << coordInMetersToRobotOrigo.x << " ; " << coordInMetersToRobotOrigo.y << "\n";

     //The found point is rotated to fit with the robots coodinate-system.
     //It is then rotated with the current angle of the robot measured from the x-axis to determine the correct position of the point compared to the robot.
     point rotatedPoint = rotatePointByAngle(M_PI_2, coordInMetersToRobotOrigo); // if the first argument for rotatePointByAngle is not
     // getTheta(cur_pose.theta) then it is in test-mode
     //std::cout << "RotatedPoint: " << rotatedPoint.x << " ; " << rotatedPoint.y << "\n";

     //The coordinates of the found paper from the robots Origin point.
     //Determined from the coordinates of the robot from its Origin + the vector from the robot centre to the found point.

     point paperPoint;
     paperPoint.x = 3 + rotatedPoint.x; //cur_pose.x
     paperPoint.y = 4 + rotatedPoint.y; //cur_pose.y
     return paperPoint;
}

visualization_msgs::Marker pointToMark(point markcalc)
{
     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     marker.header.frame_id = "/odom";
     marker.header.stamp = ros::Time();

     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "paper_pose";
     marker.id = 0;

     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     marker.type = visualization_msgs::Marker::CUBE;

     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::ADD;

     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     marker.pose.position.x = markcalc.x;
     marker.pose.position.y = markcalc.y;
     marker.pose.position.z = 0;

     marker.pose.orientation.w = 1.0;
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = 0.1;
     marker.scale.y = 0.1;
     marker.scale.z = 0.1;

     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 1.0;
     marker.color.g = 0.0;
     marker.color.b = 0.0;
     marker.color.a = 1.0;

     marker.lifetime = ros::Duration(60);

     iterationCount++;
     return marker;
}
