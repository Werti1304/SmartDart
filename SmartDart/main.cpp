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

  //medianBlur(finalMask, finalMask, 5);
  //fastNlMeansDenoising(finalMask, finalMask, 10, 7, 21);
 
  // Sharpen image
  /*Mat sharpening_kernel = (Mat_<double>(3, 3) << -1, -1, -1,
    -1, 9, -1,
    -1, -1, -1);*/
  //filter2D(finalMask, finalMask, -1, sharpening_kernel);

  Mat result;
  bitwise_and(inputImage, inputImage, result, finalMask);

  imshow(windowNameMaskRed1, mask_Color_red_1);
  imshow(windowNameMaskRed2, mask_Color_red_2);
  imshow(windowNameMaskGreen, mask_Color_green_1);
  imwrite("/home/pi/Desktop/FinalMask.jpg", finalMask);
  imwrite("/home/pi/Desktop/Result.jpg", result);
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

template<class T>
void doListStats(list<T> list)
{
  T min = numeric_limits<T>::max();
  T max = 0;
  double sum;

  for (auto i : list)
  {
    if (i < min)
    {
      min = i;
    }
    if (i > max)
    {
      max = i;
    }
    sum += i;
  }

  T valCount = list.size();
  T mean = sum / valCount;

  std::cout << "Values: " << valCount << "\nSum: " << sum << "\nMean: " << mean << "\nMin: " << min << "\nMax: " << max << endl;
}

Mat src_gray;
int thresh = 50;
RNG rng(12345);
int minArea = 100;
string contourWindow = "Contours";
string cannyWindow = "Canny Edge Detection";

list<int> executionTimeList;
int executionNum = 1000;

static void thresh_callback(int, void*)
{ 
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  for (int i = 0; i < executionNum; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();

    Canny(src_gray, canny_output, thresh, thresh * 2);

    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    auto finish = std::chrono::high_resolution_clock::now();

    auto microseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
    auto executionTime = microseconds.count();
    std::cout << executionTime << "ms\n";
    executionTimeList.push_back(executionTime);
  }

  Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); i++)
  {
    if (contourArea(contours.at(i)) > minArea)
    {
      Scalar color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
    }
  }

  doListStats(executionTimeList);

  imshow(contourWindow, drawing);
  imshow(cannyWindow, canny_output);
}

#define TESTFUNC 0
void testFunc()
{
  Mat src = imread("/home/pi/Desktop/Result.jpg");
  //src = imread("/home/pi/Desktop/TestImage.jpg");

  createResizedWindow(contourWindow);
  createResizedWindow(cannyWindow);

  cvtColor(src, src_gray, COLOR_BGR2GRAY);
  blur(src_gray, src_gray, Size(3, 3));

  const char* source_window = "Source";
  showImgResized(source_window, src);
  const int max_thresh = 255;
  const int maxArea = 500;
  createTrackbar("Canny thresh:", source_window, &thresh, max_thresh, thresh_callback);
  createTrackbar("MinArea:", source_window, &minArea, maxArea, thresh_callback);
  thresh_callback(0, 0);

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
  //defInputImage = imread("/home/pi/Desktop/TestImage5.jpg"); // Init inputImage
  defInputImage = imread("/home/pi/Desktop/TestImage.jpg"); // Init inputImage
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
