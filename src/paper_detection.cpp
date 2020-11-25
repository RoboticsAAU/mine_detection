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
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);
double degreesToRadians (double angleDegrees)
{
return angleDegrees*M_PI/180;
}

point pixelsToMeters ( point coordInPixels , double length )
{
     point coordInMeters;
     coordInMeters.x = coordInPixels.x*(length/1920);
     coordInMeters.y = coordInPixels.y*(length/1920);
     return coordInMeters;
}

turtlesim::Pose cur_pose;

 int main( int argc, char** argv )
 {
      ros::init(argc,argv, "mine_detector");
      ros::NodeHandle n;
      sub_pose = n.subscribe("/turtle1/pose", 10, &poseCallback);
  /*  VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

 int iLowH = 0;
 int iHighH = 179;

 int iLowS = 0; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;

 //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

 createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

 createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

 int iLastX = -1; 
 int iLastY = -1;

 //Capture a temporary image from the camera
 Mat imgTmp;
 cap.read(imgTmp); 

 //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

   Mat imgHSV;

  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgThresholded);

  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000)
  {
   //calculate the position of the ball
   double posX = dM10 / dArea;
   double posY = dM01 / dArea;        
   */
        
  // Calculate the lenth of the cameras view on a surface depending on the cameras distance to the surface
  double FOV = 78;
  double distFromgroundCam = 0.35;
  double halfFOV = degreesToRadians (FOV/2);
  double B = M_PI-M_PI_2-halfFOV;
  double a = (distFromgroundCam / sin(B))*sin(halfFOV);
  double alpha = atan2(9,16);
  double length = 2*cos(alpha)*a;
  double width = 2*sin(alpha)*a;

  point coordInPixel; //Use the coordinates from the camera, the current values are for testing only.
  coordInPixel.x = 450; 
  coordInPixel.y = 543;
  // convert from length in pixels to length in meters
  point coordInMeters = pixelsToMeters(coordInPixel,length);
  cout << length << " : " << width << "\n";
  cout << coordInMeters.x << " ; " << coordInMeters.y;
  coordInMeters.y = -coordInMeters.y; // convert from the cameras coordinate system with a reverted y-axis to the turtlebots normal coordinate system.
  point robotCenterToCam; // We find the vector from the middle of the robot to the middle of the camera. 
  robotCenterToCam.x = 0; 
  robotCenterToCam.y = 0.21;
  point camToOrigo;
  camToOrigo.x = -1/2*length;
  camToOrigo.y = 1/2*width;
  point robotToOrigo;
  robotToOrigo.x = robotCenterToCam.x + camToOrigo.x;
  robotToOrigo.y = robotCenterToCam.y + camToOrigo.y;
  point coordInMetersToRobotOrigo;
  coordInMetersToRobotOrigo.x = coordInMeters.x - robotToOrigo.x;
  coordInMetersToRobotOrigo.y = coordInMeters.y - robotToOrigo.y;

  
   /*
  imshow("Thresholded Image", imgThresholded); //show the thresholded image

  imgOriginal = imgOriginal + imgLines;
  imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
     */
   return 0;
}
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message)
{
    

    cur_pose.x = pose_message ->x;
    cur_pose.y = pose_message ->y;
    cur_pose.theta = pose_message -> theta;
}

// #include <iostream>
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"

// using namespace cv;
// using namespace std;

// int main(int argc, char** argv){
//     VideoCapture cap(0);

//     if(!cap.isOpened()){
//         cout << "cannot open the webcam" << endl;
//         return -1;
//     }

//     namedWindow("Control", CV_WINDOW_AUTOSIZE);

//     int iLowH = 0;
//     int iHighH = 179;

//     int iLowS = 0;
//     int iHighS = 255;

//     int iLowV = 0;
//     int iHighV = 255;

//     cvCreateTrackbar("LowH", "Control", &iLowH, 179);
//     cvCreateTrackbar("HighH", "Control", &iHighH, 179);

//     cvCreateTrackbar("LowS", "Control", &iLowS, 255);
//     cvCreateTrackbar("HighS", "Control", &iLowS, 255);

//     cvCreateTrackbar("LowV", "Control", &iLowV, 255);
//     cvCreateTrackbar("HighV", "Control", &iHighV, 255);

//     while(true){
//     Mat imgOriginal;

//     bool bSuccess = cap.read(imgOriginal);

//     if(!bSuccess){

//         cout << "Cannot read a fram from video stream" << endl;
//         break;
//     }    

//     Mat imgHSV;

//     cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

//     Mat imgThresholded;

//     inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

//      //morphological opening (removes small objects from the foreground to reduce noise)
//     erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
//     //adds foreground
//     dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

//     //morphological opening (removes small objects from the foreground to reduce noise)
//     dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
//     //adds foreground
//     erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));


//  int iLastX = -1; 
//  int iLastY = -1;

//   //Capture a temporary image from the camera
//  Mat imgTmp;
//  cap.read(imgTmp); 

//  Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;

//   Moments oMoments = moments(imgThresholded);

//  double dM01 = oMoments.m01;
//   double dM10 = oMoments.m10;
//   double dArea = oMoments.m00;

//   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
//   if (dArea > 100)
//   {
//    //calculate the position of the ball
//    int posX = dM10 / dArea;
//    int posY = dM01 / dArea;        
        
//    if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
//    {
//     //Draw a red line from the previous point to the current point
//     line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
//    }

//    iLastX = posX;
//    iLastY = posY;
//   }

//      //imgThresholded = imgThresholded + imgLines;
//     imshow("Threshholded Image", imgThresholded);

//  imgOriginal = imgOriginal + imgLines;
//     imshow("Original", imgOriginal);



//         if(waitKey(30)==27){
//             cout << "esc key is pressed by user" << endl;
//             break;
//         }
//     }

//     return 0;

// }