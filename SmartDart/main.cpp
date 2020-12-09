#include <iostream>
#include "opencv2/opencv.hpp"
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

Mat automatedColorTest()
{
  const string windowNameMaskRed1 = "Mask Red 1";
  const string windowNameMaskRed2 = "Mask Red 2";
  const string windowNameMaskGreen = "Mask Green";
  const string windowNameMaskFinal = "Mask Final";

  createResizedWindow(windowNameMaskRed1);
  createResizedWindow(windowNameMaskRed2);
  createResizedWindow(windowNameMaskGreen);
  createResizedWindow(windowNameMaskFinal);

  createResizedWindow(windowNameOutput);

  cvtColor(inputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

  // Old values without histogram equalization
  //Scalar min_Color_red_1_low = Scalar(0, 110, 30);
  //Scalar max_Color_red_1_high = Scalar(10, 255, 255);

  //Scalar min_Color_red_2_low = Scalar(170, 90, 30);
  //Scalar max_Color_red_2_high = Scalar(180, 255, 255);

  //Scalar min_Color_green_1_low = Scalar(60, 100, 5);
  //Scalar min_Color_green_1_high = Scalar(90, 255, 255);

  Scalar min_Color_red_1_low = Scalar(0, 50, 10);
  Scalar max_Color_red_1_high = Scalar(10, 255, 255);

  Scalar min_Color_red_2_low = Scalar(170, 50, 10);
  Scalar max_Color_red_2_high = Scalar(180, 255, 255);

  Scalar min_Color_green_1_low = Scalar(50, 30, 0);
  Scalar min_Color_green_1_high = Scalar(90, 255, 130);

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

  return result;
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

#define TESTFUNC 1
void testFunc()
{
  /// Histogram Equalization GRAY
  //cvtColor(inputImage, inputImage, COLOR_BGR2GRAY);

  //Mat output;
  //equalizeHist(inputImage, output);

  //string outputWindowName = "Equalized Image (Grayscale)";
  //createResizedWindow(outputWindowName);
  //imshow(outputWindowName, output);

  /// Histogram Equalization COLOR
  Mat ycrcb;

  cvtColor(inputImage, ycrcb, COLOR_BGR2YCrCb);

  vector<Mat> channels;
  split(ycrcb, channels);

  equalizeHist(channels[0], channels[0]);

  Mat result;
  merge(channels, ycrcb);

  cvtColor(ycrcb, result, COLOR_YCrCb2BGR);

  string outputWindowName = "Equalized Image (Colored)";
  createResizedWindow(outputWindowName);
  imshow(outputWindowName, result);

  inputImage = result;
  inputImage = automatedColorTest();

  //Mat outputDenoised;
  //fastNlMeansDenoisingColored(inputImage, outputDenoised);

  //string windowOutputDenoised = "Output Denoised";
  //createResizedWindow(windowOutputDenoised);
  //imshow(windowOutputDenoised, outputDenoised);

  /// Hough Circle detection
//  Mat gray;
//  cvtColor(inputImage, gray, COLOR_BGR2GRAY);
//  medianBlur(gray, gray, 5);
//  vector<Vec3f> circles;
//  HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
//    gray.rows / 16,  // change this value to detect circles with different distances to each other
//    100, 30, 10, 100 // change the last two parameters
//// (min_radius & max_radius) to detect larger circles
//);
//  for (size_t i = 0; i < circles.size(); i++)
//  {
//    Vec3i c = circles[i];
//    Point center = Point(c[0], c[1]);
//    // circle center
//    circle(inputImage, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
//    // circle outline
//    int radius = c[2];
//    circle(inputImage, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
//  }
//
//  string windowCircle = "Circles";
//  createResizedWindow(windowCircle);
//  imshow(windowCircle, inputImage);
}

void reset(string windowNameInput)
{
  destroyAllWindows();
  createResizedWindow(windowNameInput);
  imshow(windowNameInput, inputImage);
}

int main(int argc, char** argv)
{
  //inputImage = imread("/home/pi/Desktop/TestImage.jpg"); // Init inputImage
  VideoCapture cap = VideoCapture(0);
  cap.set(CAP_PROP_XI_FRAMERATE, 1);
  cap.set(CAP_PROP_FRAME_WIDTH, 2592);
  cap.set(CAP_PROP_FRAME_HEIGHT, 1944);
  cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
  cap.read(inputImage);
  cvtColor(inputImage, inputImage, COLOR_RGB2BGR); // Needed for VideoCapture b/c OpenCV is dumb

  if(inputImage.data == nullptr)
  {
    std::cout << "Couldn't find image, closing!";
    return -1;
  }

  string windowNameInput = "Input";

  if (TESTFUNC)
  {
    reset(windowNameInput);
    testFunc();
    waitKey(0);
    return 3;
  }

  for (;;)
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
