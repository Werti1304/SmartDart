#include <iostream>
#include "opencv2/opencv.hpp"
#include "HSVTrackbar.h"
#include <thread>
#include <climits>

#include "DartAreas.h"
#include "opencv2/photo.hpp"
#include "WindowHelper.h"
#include "ImageStacker.h"
#include "Resources.h"
#include "Types.h"

//#include "ShapeDetect.h"

using namespace cv;
using namespace std;

/// Most important commands:
/// imread, imshow, namedWindow, waitKey

#define RaspiWidth 1280
#define RaspiHeight 720
#define RaspiPath "/home/pi/Desktop/"

WindowHelper win(RaspiPath, RaspiHeight / 2, RaspiWidth / 2);

string windowNameInput = "Input";

string windowNameMask = "Mask";
string windowNameOutput = "Output";

Mat defInputImage;
Mat inputImageHsv;
int hsvLowValues[3] = { 0, 0, 0 }; // Default values
int hsvHighValues[3] = { 255, 255, 255 }; // Default values

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

// Helper for param packs
template <typename... T> struct param_pack {};

template<typename ...ColorFiltersT>
Mat automatedColorTest(const Mat& inputImage = defInputImage, ColorFiltersT... colorFiltersParam) // ... allows unlimited filters
{
  // Assert that all passed params are the right type
  //static_assert(std::is_same<param_pack<const HSVColorFilter>, param_pack<ColorFiltersT...>>::value, "Arguments must be HSVColorFilter.");
  HSVColorFilter colorFilters[]{ colorFiltersParam... };

  imshow(windowNameInput, inputImage);

  const string windowNameMaskRed1 = "Mask Red 1";
  const string windowNameMaskRed2 = "Mask Red 2";
  const string windowNameMaskGreen = "Mask Green";
  const string windowNameMaskFinal = "Mask Final";

  win.namedWindowResized(windowNameMaskRed1);
  win.namedWindowResized(windowNameMaskRed2);
  win.namedWindowResized(windowNameMaskGreen);
  win.namedWindowResized(windowNameMaskFinal);

  win.namedWindowResized(windowNameOutput);
  
  cvtColor(inputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

  Mat finalMask = Mat::zeros(inputImageHsv.rows, inputImageHsv.cols, CV_8UC1);
  for(HSVColorFilter colorFilter : colorFilters)
  {
    Mat tmpMask;
    inRange(inputImageHsv, colorFilter.min_Color, colorFilter.max_Color, tmpMask);
    bitwise_or(finalMask, tmpMask, finalMask);
  }

  //medianBlur(finalMask, finalMask, 5);
  //fastNlMeansDenoising(finalMask, finalMask, 10, 7, 21);
 
  // Sharpen image
  /*Mat sharpening_kernel = (Mat_<double>(3, 3) << -1, -1, -1,
    -1, 9, -1,
    -1, -1, -1);*/
  //filter2D(finalMask, finalMask, -1, sharpening_kernel);

  Mat result = Mat::zeros(inputImage.size(), CV_8UC1);
  bitwise_and(inputImage, inputImage, result, finalMask);

  //imwrite("/home/pi/Desktop/FinalMask.jpg", finalMask);
  //imwrite("/home/pi/Desktop/Result.jpg", result);
  imshow(windowNameMaskFinal, finalMask);
  imshow(windowNameOutput, result);

  return finalMask;
  return result;
}

Mat histogramEqualizationColored(Mat inputImage = defInputImage)
{
  imshow(windowNameInput, inputImage);

  Mat ycrcb;

  cvtColor(inputImage, ycrcb, COLOR_BGR2YCrCb);

  vector<Mat> channels;
  split(ycrcb, channels);

  equalizeHist(channels[0], channels[0]);

  Mat result;
  merge(channels, ycrcb);

  cvtColor(ycrcb, result, COLOR_YCrCb2BGR);

  string outputWindowName = "Equalized Image (Colored)";
  win.namedWindowResized(outputWindowName);
  imshow(outputWindowName, result);

  return result;
}

Mat automateErode(Mat inputImage = defInputImage)
{
  Mat src_gray;
  if(inputImage.channels() > 1)
  {
    cvtColor(inputImage, src_gray, COLOR_BGR2GRAY);
  }
  else
  {
    src_gray = inputImage;
  }

  win.imgshowResized("Denoised img", src_gray);

  Mat eroded_image;

  erode(src_gray, eroded_image, Mat());

  Mat result = src_gray - eroded_image;

  threshold(result, result, 1, 255, THRESH_BINARY);

  //Canny(src_gray, canny_output, cannyParam, cannyParam * 2);

  return result;
}

vector<vector<Point>> automatedContours(Mat src, Mat& drawing, int minPerimeter = 100)
{
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy; // Not needed, because of retrieve-mode (RETR_EXTERNAL)

  findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  RNG rng(time(0)); // RNG with seed of current time

  drawing = Mat::zeros(src.size(), CV_8UC3);
  vector<vector<Point>> contoursFiltered;
  for (size_t i = 0; i < contours.size(); i++)
  {
    auto cContour = contours.at(i);
    if (arcLength(cContour, true) > minPerimeter)
    {
      contoursFiltered.push_back(cContour);

      Scalar color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      drawContours(drawing, contours, static_cast<int>(i), color, 2, LINE_8, hierarchy, 0);
    }
  }

  return contoursFiltered;
}

#pragma region TestFunctions
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
    auto epsilon = 0.1 * arcLength(currentContour, true);
    approxPolyDP(currentContour, approx_curve, epsilon, true);
    auto perimeter = arcLength(approx_curve, true);

    //if (contourArea(contours.at(i)) > minArea)
    if (perimeter > minArea)
    {
      cout << perimeter << "\n";
      Scalar color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      
      drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
    }
  }

  imshow(contourWindow, drawing);
  imshow(cannyWindow, canny_output);
}

void cannyTest(Mat inputImage = defInputImage)
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

void colorTest(Mat inputImage = defInputImage)
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
    char ch = waitKey(0);
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
  Mat input = defInputImage;
  Mat eqHistogram = histogramEqualizationColored(input);
  Mat automatedColorRed = automatedColorTest(eqHistogram, Resources::red_1_histogram, Resources::red_2_histogram);
  Mat automatedColorGreen = automatedColorTest(eqHistogram, Resources::green_histogram );
  Mat cannyOutputRed = automateErode(automatedColorRed);
  Mat cannyOutputGreen = automateErode(automatedColorGreen);

  Mat drawingGreen;
  auto contoursGreen = automatedContours(cannyOutputGreen, drawingGreen);
  Mat drawingRed;
  auto contoursRed = automatedContours(cannyOutputRed, drawingRed);

  auto dartAreasGreen = DartArea::calculateAreas(contoursGreen);
  DartArea::markAreas(drawingGreen, dartAreasGreen, 3, Scalar(0, 255, 0), 3);
  auto dartAreasRed = DartArea::calculateAreas(contoursRed);
  DartArea::markAreas(drawingRed, dartAreasRed, 3, Scalar(0, 0, 255), 3);

  //win.imgshowResized("Contoured red", cannyOutputRed);
  win.imgshowResized("Contoured green", drawingGreen);
  win.imgshowResized("Contoured red", drawingRed);

  Mat contouredResult = drawingGreen + drawingRed;
  win.imgshowResized("Contoured Result", contouredResult);

  //Mat result = cannyOutputRed + cannyOutputGreen;
  //win.imgshowResized("Result", result);

  /*imwrite("/home/pi/Desktop/ContouredRed.jpg", cannyOutputRed);
  imwrite("/home/pi/Desktop/ContouredGreen.jpg", cannyOutputGreen);
  imwrite("/home/pi/Desktop/Contoured.jpg", result);*/

  //win.imgshowResized("Eroded red", cannyOutputRed);
  //win.imgshowResized("Eroded green", cannyOutputGreen);

  //Mat markedImgRed = Mat::zeros(cannyOutputRed.size(), CV_32FC1);;
  //Mat markedImgGreen = Mat::zeros(cannyOutputGreen.size(), CV_32FC1);;

  //cvtColor(cannyOutputRed, markedImgRed, COLOR_GRAY2BGR);
  //cvtColor(cannyOutputGreen, markedImgGreen, COLOR_GRAY2BGR);

  //DartAreas dartAreasRed(markedImgRed, contours);
  //DartAreas dartAreasGreen(markedImgGreen, contours);

  //markedImgRed = dartAreasRed.markAreas(5, Scalar(0, 255, 0), 5);
  //markedImgGreen = dartAreasGreen.markAreas(5, Scalar(0, 255, 0), 5);

  //win.imgshowResized("Marked img red", markedImgRed);
  //win.imgshowResized("Marked img green", markedImgGreen);
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
    case 'a':
      automatedColorTest(defInputImage, Resources::red_1_default, Resources::red_2_default, Resources::green_default);
      waitKey(0);
      break;
    case 'b':
      colorTest(histogramEqualizationColored());
      break;
    case 'c':
      {
        result = histogramEqualizationColored(badQualityImage);
        result = automatedColorTest(result, Resources::red_1_histogram, Resources::red_2_histogram, Resources::green_histogram);
        result = automateErode(result);
        
        waitKey(0);
      }
      break;
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
