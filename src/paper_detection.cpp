#include "ros/ros.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <turtlesim/Pose.h>


using namespace cv;
using namespace std;

ros::Publisher point_pub;
ros::Subscriber sub_pose;

class point
{
public:
     double x;
     double y;
};

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);
double degreesToRadians(double angleDegrees);
point pixelsToMeters(point coordInPixels, double length);
point rotatePointByAngle(double angle, point coord);
point convertCoordinatesOfPoint(vector<Point> coord);

turtlesim::Pose cur_pose;

int main(int argc, char **argv)
{

     int boundColour[] = {0, 0, 255};
     int contourColour[] = {0, 255, 0};

     ros::init(argc, argv, "mine_detector");
     ros::NodeHandle n;
     sub_pose = n.subscribe("/turtle1/pose", 10, &poseCallback);

     VideoCapture cap(0); //Capture the video from webcam.

     if (!cap.isOpened()) //If not success, exit program.
     {
          cout << "Cannot open the web cam" << endl;
          return -1;
     }

     namedWindow("Control", CV_WINDOW_AUTOSIZE); //Create a window called "Control".

     int iLowH = 0;
     int iHighH = 179;

     int iLowS = 0;
     int iHighS = 255;

     int iLowV = 0;
     int iHighV = 255;

 
     //Create trackbars in "Control" window.
     createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
     createTrackbar("HighH", "Control", &iHighH, 179);

     createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
     createTrackbar("HighS", "Control", &iHighS, 255);

     createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
     createTrackbar("HighV", "Control", &iHighV, 255);

     int iLastX = -1;
     int iLastY = -1;

     //Capture a temporary image from the camera.
     Mat imgTmp;
     cap.read(imgTmp);

     //Create a black image with the size as the camera output.
     Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);
     ;

     while (true)
     {
          Mat imgOriginal;

          bool bSuccess = cap.read(imgOriginal); //Read a new frame from video.

          if (!bSuccess) //If not success, break loop.
          {
               cout << "Cannot read a frame from video stream" << endl;
               break;
          }

          Mat imgHSV;

          cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV.

          Mat imgThresholded;

          inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image.

          //Morphological opening (removes small objects from the foreground).
          erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
          dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

          //Morphological closing (removes small holes from the foreground).
          dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
          erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


     
     vector<vector<Point>> contours;
  
     findContours(imgThresholded, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));

     vector<Rect> boundbox(contours.size());

 
     for(size_t i = 0; i < contours.size(); i++){
          approxPolyDP( Mat(contours[i]), contours[i], 3, true);
          boundbox[i] = boundingRect(contours[i]);
     }

     for( size_t i = 0; i < contours.size(); i++ ){
          rectangle( imgOriginal, boundbox[i].tl(), boundbox[i].br(), (boundColour[0], boundColour[1], boundColour[2]), 2, 8, 0 );    
     }     
     
     drawContours(imgOriginal, contours, -1, (contourColour[0], contourColour[1], contourColour[2]), 3);
     imshow("Thresholded Image", imgThresholded); //show the thresholded image
     imshow("Original", imgOriginal); //show the original image

     vector<Point> rectCenter; //current boundingbox center coordinates

     vector<int> rectSurface; //surface area of boundingbox last frame
     vector<int> lastRectSurface; //surface area of boundingbox last frame

     for( size_t i = 0; i == boundbox.size(); i++ ){//Calculate surface area of bounding boxes currently on the screen
          rectSurface[i] = boundbox[i].width * boundbox[i].height;
          if (rectSurface[i] < lastRectSurface[i]){ //if bounding rectangle is smaller than last frame save coordinates
               rectCenter[i] = {boundbox[i].x, boundbox[i].y};
          }
     }
     
     for( size_t i = 0; i == boundbox.size(); i++ ){//Save bounding rectangle surface area
          lastRectSurface[i] = rectSurface[i];
     }


        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
          cout << "esc key is pressed by user" << endl;
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

//Converts degrees to radians.
double degreesToRadians(double angleDegrees)
{
     return angleDegrees * M_PI / 180;
}

//Converts a pioint from pixels to meters, based on the ratio between side length and number of pixels.
point pixelsToMeters(point coordInPixels, double length)
{
     point coordInMeters;
     coordInMeters.x = coordInPixels.x * (length / 1920);
     coordInMeters.y = coordInPixels.y * (length / 1920);
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

point convertCoordinatesOfPoint(vector<Point> coord)
{
     // Changable variables: Diagonal FOV of the camera, and the camera distance to the ground.
     double FOV = 78;
     double distFromGroundCam = 0.35;

     //The following determines the measurements (length and width) of the area that the camera projects.
     double halfFOV = degreesToRadians(FOV / 2);
     double B = M_PI - M_PI_2 - halfFOV;
     double a = (distFromGroundCam / sin(B)) * sin(halfFOV);
     double alpha = atan2(9, 16);

     double length = 2 * cos(alpha) * a;
     double width = 2 * sin(alpha) * a;
     cout << "Dimensions: " << length << " ; " << width << "\n";

     //The coordinates of the found point in pixels.
     point coordInPixel; //Use the coordinates that will be published, the current values are for testing only.
     coordInPixel.x = coord.x;
     coordInPixel.y = coord.y;

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
     cout << "camOrigoToCamCenter: " << camOrigoToCamCenter.x << " ; " << camOrigoToCamCenter.y << "\n";

     //The vector from the projected area's Origo to the center of the robot.
     //This is done to shift the coodinate-system of the camera to a coodinate-system with Origo in the robot's centre.
     point camOrigoToRobot;
     camOrigoToRobot.x = camCenterToRobotCenter.x + camOrigoToCamCenter.x;
     camOrigoToRobot.y = camCenterToRobotCenter.y + camOrigoToCamCenter.y;
     cout << "camOrigoToRobot: " << camOrigoToRobot.x << " ; " << camOrigoToRobot.y << "\n";

     //The vector of the found point from the robot centre (in meters).
     point coordInMetersToRobotOrigo;
     coordInMetersToRobotOrigo.x = coordInMeters.x - camOrigoToRobot.x;
     coordInMetersToRobotOrigo.y = coordInMeters.y - camOrigoToRobot.y;
     cout << "coordInMetersToRobotOrigo: " << coordInMetersToRobotOrigo.x << " ; " << coordInMetersToRobotOrigo.y << "\n";

     //The found point is rotated to fit with the robots coodinate-system.
     //It is then rotated with the current angle of the robot measured from the x-axis to determine the correct position of the point compared to the robot.
     point rotatedPoint = rotatePointByAngle(0, coordInMetersToRobotOrigo);
     cout << "RotatedPoint: " << rotatedPoint.x << " ; " << rotatedPoint.y << "\n";

     //The coordinates of the found paper from the robots Origin point.
     //Determined from the coordinates of the robot from its Origin + the vector from the robot centre to the found point.
     point paperPoint;
     paperPoint.x = cur_pose.x + rotatedPoint.x;
     paperPoint.y = cur_pose.y + rotatedPoint.y;
     cout << "Paperpoint: " << paperPoint.x << " ; " << paperPoint.y << "\n";
     return paperPoint;
}