#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

int main (int argc, char** argv){
    int input = 0;
    cout << "Do you want to detect using the Camera, press 1 or load in a image, press 2,\n";
    cin >> input;
    if (input == 2)
        {
        
        // Hvis vi skal teste programmet uden selve kameraet at et billede føres ind ved brug af denne kommando
        Mat img = imread("D:/My OpenCV Website/Lady with a Guitar.jpg", IMREAD_COLOR);
            if(img.empty())
            {
            std::cout << "Could not read the image: " << std::endl;
            return 1;
            }
        }

    else 
        {
            VideoCapture cap(0); //Capture the video from webcam
            if( !cap.isOpened() ) //if not successful program will shut down
            {
            cout << "Cannot open the webcam" << endl;
            return -1;
            } 

            while(true)
            {
                bool bSuccess = cap.read(imgOriginal); //reads a new frame from camera
                if (!bSuccess)//if image cant be read, break the loop 
                { 
                cout << "Cannot read frame from video stream" << endl;
                break;
                }
            }
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
        Mat imgOriginal = Mat img;
        Mat imgHSV; //make an empty array

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //converts RGB colour to Hue, Saturation, Value and puts it intp imgHSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Decides wether a pixel is within the threshold or not

//Erosion og dilation skal evalueres i nødvendighed for optimering
        //morphological opening (removes small objects from the foreground to reduce noise)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        //adds foreground
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

        //morphological opening (removes small objects from the foreground to reduce noise)
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        //adds foreground
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

        imshow("Thresholded image", imgThresholded); //shows the tresholded image

        imshow("Original", imgOriginal); //shows the original image

        if(waitKey(30) == 27){
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }    
    return 0;
}



