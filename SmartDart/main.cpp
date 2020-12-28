#include <iostream>
#include "opencv2/opencv.hpp"
#include "HSVTrackbar.h"
#include <thread>
#include <climits>
#include "opencv2/photo.hpp"

//#include "ShapeDetect.h"

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

void showImgResized(string name, Mat img)
{
  createResizedWindow(name);
  imshow(name, img);
}

string windowNameMask = "Mask";
string windowNameOutput = "Output";

Mat defInputImage;
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
  bitwise_and(defInputImage, defInputImage, result, colorFilteredMask);

  imshow(windowNameMask, colorFilteredMask);
  imshow(windowNameOutput, result);
}

enum ColorTestAdjustment
{
  Default,
  HistogramEqualized
};

Mat automatedColorTest(ColorTestAdjustment colorTestAdjustment, Mat inputImage = defInputImage)
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

  Scalar min_Color_red_1_low;
  Scalar max_Color_red_1_high;
  Scalar min_Color_red_2_low;
  Scalar max_Color_red_2_high;
  Scalar min_Color_green_1_low;
  Scalar min_Color_green_1_high;

  switch (colorTestAdjustment)
  {
  default:
  case Default:
    // Old values without histogram equalization
    min_Color_red_1_low = Scalar(0, 110, 30);
    max_Color_red_1_high = Scalar(10, 255, 255);

    min_Color_red_2_low = Scalar(170, 90, 30);
    max_Color_red_2_high = Scalar(180, 255, 255);

    min_Color_green_1_low = Scalar(60, 100, 5);
    min_Color_green_1_high = Scalar(90, 255, 255);
    break;
  case HistogramEqualized:
    // Color-Values adjusted to histogram equalization
    min_Color_red_1_low = Scalar(0, 50, 10);
    max_Color_red_1_high = Scalar(10, 255, 255);

    min_Color_red_2_low = Scalar(170, 50, 10);
    max_Color_red_2_high = Scalar(180, 255, 255);

    min_Color_green_1_low = Scalar(50, 30, 0);
    min_Color_green_1_high = Scalar(90, 255, 130);
    break;
  }

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
  imwrite("/home/pi/Desktop/FinalMask.jpg", finalMask);
  imshow(windowNameMaskFinal, finalMask);
  imshow(windowNameOutput, result);

  return result;
}

Mat histogramEqualizationColored(Mat inputImage = defInputImage)
{
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

  return result;
}

int colorTest()
{
  cvtColor(defInputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

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

#define TESTFUNC 0
void testFunc()
{
  Mat src_gray = imread("/home/pi/Desktop/FinalMask.jpg");

  threshold(src_gray, src_gray, 127, 255, 0);
  blur(src_gray, src_gray, Size(3, 3));
  
  int thresh = 400;
  RNG rng(12345);

  Mat src_gray_denoised;
  fastNlMeansDenoising(src_gray, src_gray_denoised, 10, 7, 21);

  Mat canny_output;
  Canny(src_gray_denoised, canny_output, thresh, thresh * 2);
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
  const auto contourAreas = new double[contours.size()];
  for (size_t i = 0; i < contours.size(); i++)
  {
    contourAreas[i] = contourArea(contours.at(i));
    printf("%.1f,", contourAreas[i]);

    Scalar color = Scalar(rng.uniform(100, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
    //contours.at(i).size();
    //contours.at(i).at(0).x;
    //contours.at(i).at(0).y;
  }
  showImgResized("Contours", drawing);
  waitKey(0);
}

void reset(string windowNameInput)
{
  destroyAllWindows();
  createResizedWindow(windowNameInput);
  imshow(windowNameInput, defInputImage);
}

int main(int argc, char** argv)
{
  defInputImage = imread("/home/pi/Desktop/TestImage5.jpg"); // Init inputImage
  //defInputImage = imread("/home/pi/Desktop/MaskGreen.jpg"); // Init inputImage
  //VideoCapture cap = VideoCapture(0);
  //cap.set(CAP_PROP_XI_FRAMERATE, 1);
  //cap.set(CAP_PROP_FRAME_WIDTH, 2592);
  //cap.set(CAP_PROP_FRAME_HEIGHT, 1944);
  //cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
  //cap.read(inputImage);
  //cvtColor(inputImage, inputImage, COLOR_RGB2BGR); // Needed for VideoCapture b/c OpenCV is dumb

  if(defInputImage.data == nullptr)
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
  Mat result;

  for (;;)
  {
    reset(windowNameInput);

    char ch = waitKey(0);
    switch (ch)
    {
    case 'a':
      automatedColorTest(Default);
      waitKey(0);
      break;
    case 'b':
      colorTest();
      break;
    case 'c':
      {
        result = histogramEqualizationColored();
        automatedColorTest(HistogramEqualized, result);
        waitKey(0);
      }
      break;
    case 't':
      testFunc();
      waitKey(0);
    case 'q':
      return 0;
    default: ;
    }
  }
  
}
