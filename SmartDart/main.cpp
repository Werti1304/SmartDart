#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Helper.h"
#include "HSVTrackbar.h"
#include <thread>
#include <climits>

using namespace cv;
using namespace std;

/// Most important commands:
/// imread, imshow, namedWindow, waitKey

#define RaspiWidth 1280
#define RaspiHeight 720

void createResizedWindow(string windowName)
{
  namedWindow(windowName, WINDOW_NORMAL); // WINDOW_NORMAL needed for resizeWindow() func
  resizeWindow(windowName, RaspiWidth / 2, RaspiHeight / 2);
}

string windowNameMask = "Mask";
string windowNameOutput = "Output";

Mat inputImage;
Mat inputImageHsv;
int hsvLowValues[3] = { 0, 0, 0 }; // Default values
int hsvHighValues[3] = { 255, 255, 255 }; // Default values

void onColorFilterChange()
{
  //// https://alloyui.com/examples/color-picker/hsv.html
  Scalar min_Color = Scalar(hsvLowValues[0], hsvLowValues[1], hsvLowValues[2]);
  Scalar max_Color = Scalar(hsvHighValues[0], hsvHighValues[1], hsvHighValues[2]);

  Mat colorFilteredMask;
  inRange(inputImageHsv, min_Color, max_Color, colorFilteredMask);

  Mat result;
  bitwise_and(inputImage, inputImage, result, colorFilteredMask);

  imshow(windowNameMask, colorFilteredMask);
  imshow(windowNameOutput, result);
}

void automatedColorTest()
{
  string windowNameMaskRed1 = "Mask Red 1";
  string windowNameMaskRed2 = "Mask Red 2";
  string windowNameMaskGreen = "Mask Green";
  string windowNameMaskFinal = "Mask Final";

  createResizedWindow(windowNameMaskRed1);
  createResizedWindow(windowNameMaskRed2);
  createResizedWindow(windowNameMaskGreen);
  createResizedWindow(windowNameMaskFinal);

  createResizedWindow(windowNameOutput);

  cvtColor(inputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

  Scalar min_Color_red_1_low = Scalar(0, 110, 30);
  Scalar max_Color_red_1_high = Scalar(10, 255, 255);

  Scalar min_Color_red_2_low = Scalar(170, 90, 30);
  Scalar max_Color_red_2_high = Scalar(180, 255, 255);

  Scalar min_Color_green_1_low = Scalar(60, 100, 5);
  Scalar min_Color_green_1_high = Scalar(90, 255, 255);

  Mat mask_Color_red_1;
  inRange(inputImageHsv, min_Color_red_1_low, max_Color_red_1_high, mask_Color_red_1);

  Mat mask_Color_red_2;
  inRange(inputImageHsv, min_Color_red_2_low, max_Color_red_2_high, mask_Color_red_2);

  Mat mask_Color_green_1;
  inRange(inputImageHsv, min_Color_green_1_low, min_Color_green_1_high, mask_Color_green_1);

  Mat finalMask;
  bitwise_or(mask_Color_red_1, mask_Color_red_2, finalMask);
  bitwise_or(finalMask, mask_Color_green_1, finalMask);

  Mat result;
  bitwise_and(inputImage, inputImage, result, finalMask);

  imshow(windowNameMaskRed1, mask_Color_red_1);
  imshow(windowNameMaskRed2, mask_Color_red_2);
  imshow(windowNameMaskGreen, mask_Color_green_1);
  imshow(windowNameMaskFinal, finalMask);
  imshow(windowNameOutput, result);
}

int colorTest()
{
  cvtColor(inputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

  // Create windows in right size
  createResizedWindow(windowNameMask);
  createResizedWindow(windowNameOutput);

  // Create trackbar for low bar of colors
  HSVTrackbar trackbarLow("HSV-Trackbar Low", hsvLowValues);

  // Create trackbar for high bar of colors
  HSVTrackbar trackbarHigh("HSV-Trackbar High", hsvHighValues);

  // First run to init the images
  onColorFilterChange();

  for(;;)
  {
    char ch = waitKey(0);
    if(ch == 'q')
    {
      return 0;
    }
    if(ch == 'c')
    {
      trackbarLow.setCallback(onColorFilterChange);
      trackbarHigh.setCallback(onColorFilterChange);
    }
    else if (ch == 'r')
    {
      trackbarLow.setCallback(nullptr);
      trackbarHigh.setCallback(nullptr);
    }
    else
    {
      onColorFilterChange();
    }
  }
}

void reset(string windowNameInput)
{
  destroyAllWindows();
  createResizedWindow(windowNameInput);
  imshow(windowNameInput, inputImage);
}

int main(int argc, char** argv)
{
  inputImage = imread("/home/pi/Desktop/TestImage.jpg"); // Init inputImage
  //VideoCapture cap = VideoCapture(0);
  //cap.set(CAP_PROP_XI_FRAMERATE, 1);
  //cap.set(CAP_PROP_FRAME_WIDTH, 2592);
  //cap.set(CAP_PROP_FRAME_HEIGHT, 1944);
  //cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
  //cap.read(inputImage);
  //cvtColor(inputImage, inputImage, COLOR_RGB2BGR); // Needed for VideoCapture b/c OpenCV is dumb

  if(inputImage.data == nullptr)
  {
    std::cout << "Couldn't find image, closing!";
    return -1;
  }

  string windowNameInput = "Input";
  for(;;)
  {
    reset(windowNameInput);
    char ch = waitKey(0);
    switch (ch)
    {
    case 'a':
      automatedColorTest();
      waitKey(0);
      break;
    case 'b':
      colorTest();
      break;
    case 'q':
      return 0;
    }
  }
}
