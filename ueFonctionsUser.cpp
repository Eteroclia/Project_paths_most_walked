#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include <iomanip>
#include "ueIntegration.h"
#include <opencv2/highgui.hpp>

calibration calibrator(0,0);
depthDetection detector(0,0);


int numDisparities = 8;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
float M = 4.4075000000000003e+01;

static void on_trackbar1( int, void* )
{
  detector.numDisparities=numDisparities*16;
  numDisparities = numDisparities*16;
}
 
static void on_trackbar2( int, void* )
{
  detector.blockSize=blockSize*2+5;
  blockSize = blockSize*2+5;
}
 
static void on_trackbar3( int, void* )
{
  detector.preFilterType=preFilterType;
}
 
static void on_trackbar4( int, void* )
{
  detector.preFilterSize=preFilterSize*2+5;
  preFilterSize = preFilterSize*2+5;
}
 
static void on_trackbar5( int, void* )
{
  detector.preFilterCap=preFilterCap;
}
 
static void on_trackbar6( int, void* )
{
  detector.textureThreshold=textureThreshold;
}
 
static void on_trackbar7( int, void* )
{
  detector.uniquenessRatio=uniquenessRatio;
}
 
static void on_trackbar8( int, void* )
{
  detector.speckleRange=speckleRange;
}
 
static void on_trackbar9( int, void* )
{
  detector.speckleWindowSize=speckleWindowSize*2;
  speckleWindowSize = speckleWindowSize*2;
}
 
static void on_trackbar10( int, void* )
{
  detector.disp12MaxDiff=disp12MaxDiff;
}
 
static void on_trackbar11( int, void* )
{
  detector.minDisparity=minDisparity;
}

int main()
{
    


    cv::namedWindow("control",cv::WINDOW_NORMAL);
    cv::resizeWindow("control",600,600);

    cv::namedWindow("depth",cv::WINDOW_NORMAL);
    cv::resizeWindow("depth",600,600);


   
    // Creating trackbars to dynamically update the StereoBM parameters
    cv::createTrackbar("numDisparities", "control", &numDisparities, 18, on_trackbar1);
    cv::createTrackbar("blockSize", "control", &blockSize, 50, on_trackbar2);
    cv::createTrackbar("preFilterType", "control", &preFilterType, 1, on_trackbar3);
    cv::createTrackbar("preFilterSize", "control", &preFilterSize, 25, on_trackbar4);
    cv::createTrackbar("preFilterCap", "control", &preFilterCap, 62, on_trackbar5);
    cv::createTrackbar("textureThreshold", "control", &textureThreshold, 100, on_trackbar6);
    cv::createTrackbar("uniquenessRatio", "control", &uniquenessRatio, 100, on_trackbar7);
    cv::createTrackbar("speckleRange", "control", &speckleRange, 100, on_trackbar8);
    cv::createTrackbar("speckleWindowSize", "control", &speckleWindowSize, 25, on_trackbar9);
    cv::createTrackbar("disp12MaxDiff", "control", &disp12MaxDiff, 25, on_trackbar10);
    cv::createTrackbar("minDisparity", "control", &minDisparity, 25, on_trackbar11);


  while (true)
  {

    // Close window using esc key
    if (cv::waitKey(1) == 27) break;

  }

  return 0;
}