#include "ros/ros.h"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher rect_cent;

int main(int argc, char **argv)
{
     vector<int> lastRectSurface; //surface area of boundingbox last frame
     vector<bool> shouldPublish;
     int boundColour[] = {0, 0, 255};
     int contourColour[] = {0, 255, 0};
     int surflimit = 250;            //surflimit defines the lower boundary, where an object will be countoured and for which a bounding box will be made
     int upperLimitOfdecrease = 100; // this defines the upper limit for the change of the size of the bounding boxes
     int surfacedif;

     cv::Mat uppermask;
     cv::Mat lowermask;

     ros::init(argc, argv, "paper_detector");
     ros::NodeHandle n;
     rect_cent = n.advertise<geometry_msgs::Point>("/rect_center", 100); //visualization_msgs::Marker /visualization_marker
     //led_pub = n.advertise<kobuki_msgs::Led>("/commands/led1", 10);

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
          ros::spinOnce(); //process odom callback.

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

          if (iLowH > iHighH)
          {
               cv::inRange(imgHSV, cv::Scalar(0, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), lowermask);
               cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(179, iHighS, iHighV), uppermask);
               imgThresholded = lowermask | uppermask;
          }
          else
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
          //shouldPublish.resize(rectCenter.size(), true);
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

                    for (size_t i = 0; i < rectCenter.size() && shouldPublish[i] == true; i++)
                    {
                         geometry_msgs::Point rectPoint;
                         rectPoint.x = rectCenter.at(i).x;
                         rectPoint.y = rectCenter.at(i).y;

                         rect_cent.publish(rectPoint);
                         //point_pub.publish(pointToMark(convertCoordinatesOfPoint(centerCoord)));
                         shouldPublish[i] = false;
                    }
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
