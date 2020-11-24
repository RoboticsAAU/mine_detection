#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv){
    VideoCapture cap(0);

    if(!cap.isOpened()){
        cout << "cannot open the webcam" << endl;
        return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE);

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    cvCreateTrackbar("LowH", "Control", &iLowH, 179);
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255);
    cvCreateTrackbar("HighS", "Control", &iLowS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255);
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    while(true){
    Mat imgOriginal;

    bool bSuccess = cap.read(imgOriginal);

    if(!bSuccess){

        cout << "Cannot read a fram from video stream" << endl;
        break;
    }    

    Mat imgHSV;

    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

     //morphological opening (removes small objects from the foreground to reduce noise)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    //adds foreground
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    //morphological opening (removes small objects from the foreground to reduce noise)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    //adds foreground
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));


    imshow("Threshholded Image", imgThresholded);
    imshow("Original", imgOriginal);



        if(waitKey(30)==27){
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;

}