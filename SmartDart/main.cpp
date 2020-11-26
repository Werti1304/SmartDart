#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Helper.h"
#include "HSVTrackbar.h"

using namespace cv;
using namespace std;

/// Most important commands:
/// imread, imshow, namedWindow, waitKey

#define RaspiWidth 1280
#define RaspiHeight 720

void showResizedImage(string windowName, Mat image)
{
  namedWindow(windowName, WINDOW_NORMAL); // WINDOW_NORMAL needed for resizeWindow() func
  resizeWindow(windowName, RaspiWidth, RaspiHeight);
  imshow(windowName, image);
}

//TODO: Make prettier (if possible)
Mat trackBarImgHSV;
Mat trackBarImgRGB;
int values[3] = {0, 255, 255 }; // Default values 
string windowColorControlPanel = "Color Control Panel";
// END TODO
static void on_trackbar(int, void*)
{
  // https://stackoverflow.com/questions/31329437/trackbar-to-choose-color-using-c-opencv
  trackBarImgHSV.setTo(Scalar(values[0], values[1], values[2]));
  cvtColor(trackBarImgHSV, trackBarImgRGB, COLOR_HSV2BGR); // BGR is standard
  imshow(windowColorControlPanel, trackBarImgRGB);
}

// https://docs.opencv.org/3.4/da/d6a/tutorial_trackbar.html
void createScalarTrackbar(string windowName, string trackbarNames[3], int* values, TrackbarCallback onCall)
{
  namedWindow(windowName);
  createTrackbar(trackbarNames[0], windowName, &values[0], 255, onCall);
  createTrackbar(trackbarNames[1], windowName, &values[1], 255, onCall);
  createTrackbar(trackbarNames[2], windowName, &values[2], 255, onCall);
}

int main(int argc, char** argv)
{
  // Init images
  trackBarImgRGB = Mat3b(100, 300, Vec3b(0, 0, 0));
  cvtColor(trackBarImgRGB, trackBarImgHSV, COLOR_BGR2HSV);

  Mat image = imread("/home/pi/Desktop/TestImage.jpg"); //capture the video from web cam

  //string trackBarNames[] = { "H", "S", "V" };
  //createScalarTrackbar(windowColorControlPanel, trackBarNames, values, on_trackbar);
  //on_trackbar(0, nullptr); // Initialize trackbar 

  int hsvValues[3] = { 255, 255, 255 };
  HSVTrackbar trackbar("Hi", hsvValues);

  Mat imageHsv;
  cvtColor(image, imageHsv, COLOR_BGR2HSV);

  // https://alloyui.com/examples/color-picker/hsv.html
  Scalar min_Color = cv::Scalar(0, 0, 0);
  Scalar max_Color = cv::Scalar(255, 255, 255);

  Mat colorFilteredMask;
  inRange(imageHsv, min_Color, max_Color, colorFilteredMask);
  
  Mat result;
  bitwise_and(image, image, result, colorFilteredMask);

  showResizedImage("Input", image);
  showResizedImage("Mask", colorFilteredMask);
  showResizedImage("Output", result);

  waitKey(0);

  return 0;
}