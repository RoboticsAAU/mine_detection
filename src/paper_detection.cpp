#include "ros/ros.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <turtlesim/Pose.h>

//using namespace cv;
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
point convertCoordinatesOfPoint(point Coord);

turtlesim::Pose cur_pose;

int main(int argc, char **argv)
{

     vector<int> lastRectSurface; //surface area of boundingbox last frame

     int boundColour[] = {0, 0, 255};
     int contourColour[] = {0, 255, 0};

     int surflimit = 50;
     int surfacedif;

     ros::init(argc, argv, "mine_detector");
     ros::NodeHandle n;

     // sub_pose = n.subscribe("/turtle1/pose", 10, &poseCallback);
     // point_pub = n.advertise<point_coords.msg>("/paper_pose", 10);

     cv::VideoCapture cap(1); //Capture the video from webcam.

     if (!cap.isOpened()) //If not success, exit program.
     {
          std::cout << "Cannot open the web cam" << endl;
          return -1;
     }

     cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //Create a window called "Control".

     int iLowH = 0;
     int iHighH = 179;

     int iLowS = 150;
     int iHighS = 255;

     int iLowV = 150;
     int iHighV = 255;

     //Create trackbars in "Control" window.
     cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
     cv::createTrackbar("HighH", "Control", &iHighH, 179);

     cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
     cv::createTrackbar("HighS", "Control", &iHighS, 255);

     cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
     cv::createTrackbar("HighV", "Control", &iHighV, 255);

     int check = 0; //used for defining how many frames to skip for defining last rectangle surface

     while (ros::ok())
     {

          

          cv::Mat imgOriginal;

          bool bSuccess = cap.read(imgOriginal); //Read a new frame from video.
          
          check++; //add 1 to check 
          //cout << "Itteration----------------------------------------------------------------------------------------" << check << endl;

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
          

          vector<vector<cv::Point>> contours;

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
               
               vector<int> rectSurface(boundbox.size());     //surface area of boundingbox last frame
               lastRectSurface.resize(boundbox.size()); //surface area of boundingbox last frame

               for (size_t i = 0; i < boundbox.size(); i++ ) { 
                    // cout << "bounding amount = " << boundbox.size() << endl;                    
                    // cout << "width = " << boundbox[i].width << endl; 
                    // cout << "heigth = " << boundbox[i].height << endl;                                        
                    
                    rectSurface[i] = boundbox[i].width * boundbox[i].height;

                    surfacedif = lastRectSurface[i] - rectSurface[i];
                    /*
                    cout << "surface = " << rectSurface[i] << endl;
                    cout << "last surface = " << lastRectSurface[i] << endl;
                    cout << "surface dif = " << surfacedif << endl;
                    cout << "surface limit = " << surflimit << endl;
                    */
                    if (surfacedif > surflimit) { //if bounding rectangle is smaller than last frame save coordinated of bounding rectangle
                         rectCenter[i] = {boundbox[i].x + (boundbox[i].width/2), boundbox[i].y + (boundbox[i].height/2)};
                         //cout << "center = " << rectCenter[i] << endl;
                    }                    
                    lastRectSurface[i] = rectSurface[i];
                    
               }

               if (rectCenter.size() != 0) 
               {
                    for (size_t i = 0; i < rectCenter.size(); i++)
                    { 
                    point centerCoord;
                    centerCoord.x = double(rectCenter.at(i).x);
                    centerCoord.y = double(rectCenter.at(i).y);

                   
                    //std::cout << rectCenter.at(i).x << " : " << rectCenter.at(i).y;
                         if (centerCoord.x && centerCoord.y != 0)
                         {
                         point paperPoint = convertCoordinatesOfPoint(centerCoord);
                         std::cout << centerCoord.x << " ; " << centerCoord.y << std::endl;
                         std::cout << "Paperpoint: " << paperPoint.x << " ; " << paperPoint.y << "\n";
                         /*//point_pub = pointCentConverted.x;
                         
                         */
                         }
                    }
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
     std::cout << "HalfFOV: " << halfFOV;
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
     point rotatedPoint = rotatePointByAngle(0, coordInMetersToRobotOrigo);
     std::cout << "RotatedPoint: " << rotatedPoint.x << " ; " << rotatedPoint.y << "\n";

     //The coordinates of the found paper from the robots Origin point.
     //Determined from the coordinates of the robot from its Origin + the vector from the robot centre to the found point.
     point paperPoint;
     paperPoint.x = 0 + rotatedPoint.x; //cur_pose.x
     paperPoint.y = 0 + rotatedPoint.y; //cur_pose.y
     return paperPoint;
} 

