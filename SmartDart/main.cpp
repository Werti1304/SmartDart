#include <iostream>
#include "opencv2/opencv.hpp"
#include "HSVTrackbar.h"
#include <thread>
#include <climits>

#include "Automation.h"
#include "DartAreas.h"
#include "WindowHelper.h"
#include "Resources.h"

using namespace cv;
using namespace std;

#define RaspiWidth 1280
#define RaspiHeight 720
#define RaspiPath "/home/pi/Desktop/"

WindowHelper win(RaspiPath, RaspiHeight / 2, RaspiWidth / 2);



#pragma region Helper
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
#pragma endregion

#pragma region TestFunctions

string windowNameInput = "Input";

string windowNameMask = "Mask";
string windowNameOutput = "Output";

Mat defInputImage;
Mat inputImageHsv;
int hsvLowValues[3] = { 0, 0, 0 }; // Default values
int hsvHighValues[3] = { 255, 255, 255 }; // Default values
void onColorFilterChange()
{
  //// https://alloyui.com/examples/color-picker/hsv.html
  const Scalar min_Color = Scalar(hsvLowValues[0], hsvLowValues[1], hsvLowValues[2]);
  const Scalar max_Color = Scalar(hsvHighValues[0], hsvHighValues[1], hsvHighValues[2]);

  Mat colorFilteredMask;
  inRange(inputImageHsv, min_Color, max_Color, colorFilteredMask);

  Mat result;
  bitwise_and(defInputImage, defInputImage, result, colorFilteredMask);

  imshow(windowNameMask, colorFilteredMask);
  imshow(windowNameOutput, result);
}

Mat src_gray;
int thresh = 50;
int minArea = 150;
string contourWindow = "Contours";
string cannyWindow = "Canny Edge Detection";
static void CannyTestThreshCallback(int, void*)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Canny(src_gray, canny_output, thresh, thresh * 2);

  findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
  RNG rng(time(0)); // RNG with seed of current time

  for (size_t i = 0; i < contours.size(); i++)
  {
    // Perimeter
    vector<Point_<int>> approx_curve;
    auto currentContour = contours.at(i);
    const auto epsilon = 0.1 * arcLength(currentContour, true);
    approxPolyDP(currentContour, approx_curve, epsilon, true);
    const auto perimeter = arcLength(approx_curve, true);

    //if (contourArea(contours.at(i)) > minArea)
    if (perimeter > minArea)
    {
      cout << perimeter << "\n";
      Scalar color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      
      drawContours(drawing, contours, static_cast<int>(i), color, 2, LINE_8, hierarchy, 0);
    }
  }

  imshow(contourWindow, drawing);
  imshow(cannyWindow, canny_output);
}

void cannyTest(const Mat& inputImage = defInputImage)
{
  imshow(windowNameInput, inputImage);

  win.namedWindowResized(contourWindow);
  win.namedWindowResized(cannyWindow);

  cvtColor(inputImage, src_gray, COLOR_BGR2GRAY);
  blur(src_gray, src_gray, Size(3, 3));

  const char* source_window = "Source";
  win.imgshowResized(source_window, inputImage);
  const int max_thresh = 255;
  const int maxArea = 500;
  createTrackbar("Canny thresh:", source_window, &thresh, max_thresh, CannyTestThreshCallback);
  createTrackbar("MinArea:", source_window, &minArea, maxArea, CannyTestThreshCallback);
  CannyTestThreshCallback(0, 0);

  waitKey(0);
}

void colorTest(const Mat& inputImage = defInputImage)
{
  imshow(windowNameInput, inputImage);

  cvtColor(inputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

  // Create windows in right size
  win.namedWindowResized(windowNameMask);
  win.namedWindowResized(windowNameOutput);

  // Create trackbar for low bar of colors
  HSVTrackbar trackbarLow("HSV-Trackbar Low", hsvLowValues);

  // Create trackbar for high bar of colors
  HSVTrackbar trackbarHigh("HSV-Trackbar High", hsvHighValues);

  // First run to init the images
  onColorFilterChange();

  for (;;)
  {
    const char ch = waitKey(0);
    if (ch == 'q')
    {
      return;
    }
    if (ch == 'c')
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

#pragma endregion

#define TESTFUNC 0
void testFunc()
{
  enum Images
  {
    Source,
    Histogram,
    MaskRed,
    MaskGreen,
    FilteredRed,
    FilteredGreen,
    ErosionRed,
    ErosionGreen,
    DrawingRed,
    DrawingGreen,
    DrawingResult,
    DartBoard,
    END
  };

  Mat image[END];

  image[Source] = defInputImage;

  Automation::histogramEqualizationColored(image[Source], image[Histogram]);

  Automation::colorFilter(image[Histogram], image[MaskRed], image[FilteredRed], Resources::red_1_histogram, Resources::red_2_histogram);
  Automation::colorFilter(image[Histogram], image[MaskGreen], image[FilteredGreen], Resources::green_histogram );

  Automation::erosion(image[FilteredRed], image[ErosionRed]);
  Automation::erosion(image[FilteredGreen], image[ErosionGreen]);

  const auto contoursGreen = Automation::contours(image[ErosionRed], image[DrawingRed], true );
  const auto contoursRed = Automation::contours(image[ErosionGreen], image[DrawingGreen],true );

  const auto dartAreasGreen = DartArea::calculateAreas(contoursGreen);
  const auto dartAreasRed = DartArea::calculateAreas(contoursRed);

  image[DrawingResult] = image[DrawingRed] + image[DrawingGreen];

  const std::list<DartArea> dartboard = DartArea::defineDartBoard(dartAreasGreen, dartAreasRed);
  DartArea::markAreas(image[DrawingResult], dartboard, 3, Scalar(0, 0, 255), 3);

  win.imgshowResized("Contoured Result", image[DrawingResult]);

  image[DartBoard] = Mat::zeros(image[Source].size(), CV_8UC3);

  const auto contours = DartArea::convertToContours(dartboard);
  RNG rng(time(0)); // RNG with seed of current time

  for (size_t i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));

    drawContours(image[DartBoard], contours, static_cast<int>(i), color, 2);
  }
  win.imgshowResized("Final Dartboard", image[DartBoard]);
}

void reset(string windowNameInput)
{
  destroyAllWindows();
  win.namedWindowResized(windowNameInput);
  imshow(windowNameInput, defInputImage);
}

int main(int argc, char** argv)
{
  string homePath = "/home/pi/Desktop/";

  Mat badQualityImage = win.imreadRel("TestImage5.jpg");
  Mat hqFinalMask = win.imreadRel("FinalMask2.jpg");
  Mat hqColorResult = win.imreadRel("Result.jpg");
  Mat hqContours = win.imreadRel("Contours.jpg");

  //defInputImage = imread("/home/pi/Desktop/TestImage5.jpg"); // Init inputImage
  defInputImage = win.imreadRel("TestImage.jpg"); // Init inputImage
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
  //reset(windowNameInput);
  //waitKey(0);
  //ImageStacker imageStacker("lol", RaspiWidth, RaspiHeight);
  //imageStacker.addImage(defInputImage);
  //imageStacker.addImage(badQualityImage);
  //imageStacker.addImage(hqFinalMask);
  //imageStacker.addImage(hqColorResult);
  //imageStacker.addImage(hqContours);
  //for (int i = 0; i < 5; i++)
  //{
  //  imageStacker.addImage(defInputImage);
  //}
  //imageStacker.addImage(defInputImage, true);
  //waitKey(0);

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
    //case 'a':
    //  Automation::automatedColorTest(defInputImage, Resources::red_1_default, Resources::red_2_default, Resources::green_default);
    //  waitKey(0);
    //  break;
    //case 'b':
    //  colorTest(Automation::histogramEqualizationColored());
    //  break;
    //case 'c':
    //  {
    //    result = Automation::histogramEqualizationColored(badQualityImage);
    //    result = Automation::automatedColorTest(result, Resources::red_1_histogram, Resources::red_2_histogram, Resources::green_histogram);
    //    result = automateErode(result);
    //    
    //    waitKey(0);
    //  }
    //  break;
    case 'd':
    {
      cannyTest(hqColorResult);
    }
    break;
    case 't':
      defInputImage = badQualityImage;
      testFunc();
      waitKey(0);
    case 'q':
      return 0;
    default: ;
    }
  }
}
