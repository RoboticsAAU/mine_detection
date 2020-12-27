#include "ros/ros.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher rect_cent;

void createTrackbars();
void publishRectPoint(vector<cv::Point> vectorCoordinates, vector<bool> shouldPub);
cv::Mat denoiseImg(cv::Mat imgThresholded);
cv::Mat defineRange(cv::Mat imgHSV, cv::Mat imgThresholded);

int iLowH = 170;
int iHighH = 179;

int iLowS = 170;
int iHighS = 255;

int iLowV = 83;
int iHighV = 255;

int main(int argc, char **argv)
{
     vector<int> lastRectSurface; //Surface area of boundingbox last frame
     vector<bool> shouldPublish;
     int boundColour[] = {0, 0, 255};   // Colours of drawn boundingrectangle in R - G - B
     int contourColour[] = {0, 255, 0}; //colours of drawn contours in R - G - B
     int surflimit = 250;               //Surface limit defines the lower boundary, where an object will be countoured and for which a bounding box will be made
     int upperLimitOfdecrease = 100;    //This defines the upper limit for the change of the size of the bounding boxes
     int surfacedif;                    //Surface difference limit for publishing check

     ros::init(argc, argv, "paper_detector");
     ros::NodeHandle n;
     rect_cent = n.advertise<geometry_msgs::Point>("/rect_center", 100); //visualization_msgs::Marker /visualization_marker

     cv::VideoCapture cap(0); //Capture the video from webcam.
     //If the webcam cannot open, it is likely due to the iindex being wrong, thus it is trying to open a webcam that is not accessible through that index.

     if (!cap.isOpened()) //If not success, exit program.
     {
          std::cout << "Cannot open the web cam" << endl;
          return -1;
     }

     cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //Create a window called "Control".

     //Create a window with trackbars for HSV values.
     createTrackbars(); //These are changeable in the window, but are defined as global variables from the start.

     while (ros::ok())
     {
          ros::spinOnce(); //Process odom callback.

          cv::Mat imgOriginal;

          bool bSuccess = cap.read(imgOriginal); //Read a new frame from video.

          if (!bSuccess) //If not success, break loop.
          {
               std::cout << "Cannot read a frame from video stream" << endl;
               break;
          }

          cv::Mat imgHSV;
          cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV.

          cv::Mat imgThresholded = defineRange(imgHSV, imgThresholded);
          imgThresholded = denoiseImg(imgThresholded); //Saves denoised image in the original thresholded image

          vector<vector<cv::Point>> contours; //Makes a 2D vector containing points

          findContours(imgThresholded, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); //finds contours from thresholded image

          vector<cv::Rect> boundbox(contours.size());
          vector<vector<cv::Point>> contours_poly(contours.size());

          for (size_t i = 0; i < contours.size(); i++)
          {
               approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
               boundbox[i] = boundingRect(contours_poly[i]);
               drawContours(imgOriginal, contours_poly, -1, (contourColour[0], contourColour[1], contourColour[2]), 3);
               rectangle(imgOriginal, boundbox[i].tl(), boundbox[i].br(), (boundColour[0], boundColour[1], boundColour[2]), 2, 8, 0);
          }

          imshow("Thresholded Image", imgThresholded); //Show the thresholded image.
          imshow("Original", imgOriginal);             //Show the original image.

          vector<cv::Point> rectCenter(boundbox.size()); //Current boundingbox center coordinates.
          shouldPublish.resize(rectCenter.size(), true);

          vector<int> rectSurface(boundbox.size()); //Surface area of boundingbox frame.
          lastRectSurface.resize(boundbox.size());  //Surface area of boundingbox last frame.

          for (size_t i = 0; i < boundbox.size(); i++)
          {
               rectSurface[i] = boundbox[i].width * boundbox[i].height;
               surfacedif = lastRectSurface[i] - rectSurface[i];
               if (surfacedif > surflimit) //If bounding rectangle is smaller than last frame save coordinated of bounding rectangle
               {
                    rectCenter[i] = {boundbox[i].x + (boundbox[i].width / 2), boundbox[i].y + (boundbox[i].height / 2)};
                    publishRectPoint(rectCenter, shouldPublish);
               }
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

//Create trackbars in "Control" window.
void createTrackbars()
{
     cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
     cv::createTrackbar("HighH", "Control", &iHighH, 179);

     cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
     cv::createTrackbar("HighS", "Control", &iHighS, 255);

     cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
     cv::createTrackbar("HighV", "Control", &iHighV, 255);
}

//Publish rectPoint x- and y-coordinates to rect_cent for the i'th point if the boolean shouldPub is true.
void publishRectPoint(vector<cv::Point> vectorCoordinates, vector<bool> shouldPub)
{
     for (size_t i = 0; i < vectorCoordinates.size() && shouldPub[i] == true; i++)
     {
          geometry_msgs::Point rectPoint;
          rectPoint.x = vectorCoordinates.at(i).x;
          rectPoint.y = vectorCoordinates.at(i).y;

          rect_cent.publish(rectPoint);

          shouldPub[i] = false;
     }
}

//Removes noise and fixes holes in binary image.
cv::Mat denoiseImg(cv::Mat imgThresholded)
{
     //Morphological opening (removes small objects from the foreground).
     erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
     dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

     //Morphological closing (removes small holes from the foreground).
     dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
     erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

     return imgThresholded;
}

//The function defines the thresholds for the HSV values.
cv::Mat defineRange(cv::Mat imgHSV, cv::Mat imgThresholded)
{
     cv::Mat uppermask;
     cv::Mat lowermask;

     if (iLowH > iHighH)
     {
          cv::inRange(imgHSV, cv::Scalar(0, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), lowermask);
          cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(179, iHighS, iHighV), uppermask);
          imgThresholded = lowermask | uppermask;
     }
     else
     {
          cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image.
     }

     return imgThresholded;
}