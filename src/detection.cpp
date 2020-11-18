#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main (int argc, char** argv){
    VideoCapture cap(0); //Capture the video from webcam

    if( !cap.isOpened() ){ //if not successful program will shut down
    cout << "Cannot open the webcam" << endl;
    return -1;
    }  

    namedWindow("control", CV_WINDOW_AUTOSIZE); //creates a windows called "Control"

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;
    //Hue, Saturation and Value intervals
    //hue is the color
    //Saturation is amount of white mixed in
    //Value is amount of black mixed in

    //create trackbars in Control window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179);
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
    //Hue 0-179

    cvCreateTrackbar("LowS", "Control", &iLowS, 255);
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
    //Saturation 0-255

    cvCreateTrackbar("LowV", "Control", &iLowV, 255);
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
    //Value 0-255

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); //reads a new frame from camera

        if (!bSuccess) { //if image cant be read, break the loop
        cout << "Cannot read frame from video stream" << endl;
        break;
        }

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //converts RGB colour to Hue, Saturation, Value

        Mat imgThreshholded;

        //inRange(imgHSV, Scalar);

    }    

}



