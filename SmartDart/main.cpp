#include <iostream>
#include "opencv2/opencv.hpp"
#include "HSVTrackbar.h"
#include <thread>
#include <climits>

#include "Automation.h"
#include "DartAreas.h"
#include "ImageStacker.h"
#include "WindowHelper.h"
#include "Resources.h"

using namespace cv;
using namespace std;

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

  _win.namedWindowResized(contourWindow);
  _win.namedWindowResized(cannyWindow);

  cvtColor(inputImage, src_gray, COLOR_BGR2GRAY);
  blur(src_gray, src_gray, Size(3, 3));

  const char* source_window = "Source";
  _win.imgshowResized(source_window, inputImage);
  const int max_thresh = 255;
  const int maxArea = 500;
  createTrackbar("Canny thresh:", source_window, &thresh, max_thresh, CannyTestThreshCallback);
  createTrackbar("MinArea:", source_window, &minArea, maxArea, CannyTestThreshCallback);
  CannyTestThreshCallback(0, 0);

  waitKey(0);
}

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

void colorTest(const Mat& inputImage = defInputImage)
{
  imshow(windowNameInput, inputImage);

  cvtColor(inputImage, inputImageHsv, COLOR_BGR2HSV); // Init inputImageHsv

  // Create windows in right size
  _win.namedWindowResized(windowNameMask);
  _win.namedWindowResized(windowNameOutput);

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

DartBoard* _dartboard;

void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
  if (event == EVENT_LBUTTONDOWN)
  {
    cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  }
}

void drawDartBoardProgressLine(DartBoard dartBoard, const Mat& source, const Mat& contours)
{
  _dartboard = &dartBoard;

  enum Images
  {
    Source,
    Contours,
    DartAreaSigPoints,
    DartBoardNeighbour,
    DartBoardSigPoints,
    DartBoardSigDoubleTriples,
    DartBoardSigBullseye,
    DartBoardCon,
    DartBoardCorners,
    DartBoardSortedMeanCorners,
    DartBoardDrawn,
    DartBoardDrawn2,
    END
  };
    
  std::array<Mat, END> images = {
    source.clone(),
    contours.clone(),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3) };

  // DartAreaSigPoints
  for (auto area : dartBoard.greenContours)
  {
    circle(images[DartAreaSigPoints], area.meanPoint, 3, _greenColor, 3);
    for (auto pt : area.significantPoints)
    {
      circle(images[DartAreaSigPoints], pt, 3, _greenColor, 3);
    }
  }
  for (auto area : dartBoard.redContours)
  {
    circle(images[DartAreaSigPoints], area.meanPoint, 3, _redColor, 3);
    for (auto pt : area.significantPoints)
    {
      circle(images[DartAreaSigPoints], pt, 3, _redColor, 3);
    }
  }

  // DartBoardNeighbour
  circle(images[DartBoardNeighbour], dartBoard.doubles[AREA_20]->meanPoint, 3, _redColor, 3);
  for (auto pt : dartBoard.doubles[AREA_20]->significantPoints)
  {
    circle(images[DartBoardNeighbour], pt, 3, _redColor, 3);
  }
  for(DartArea* neighbour : dartBoard.doubles[AREA_20]->neighbours)
  {
    circle(images[DartBoardNeighbour], neighbour->meanPoint, 3, _greenColor, 3);
    for (auto pt : neighbour->significantPoints)
    {
      circle(images[DartBoardNeighbour], pt, 3, _greenColor, 3);
    }
  }

  // DartBoardSigPoints
  for (auto i = 0; i < 20; i++)
  {
    circle(images[DartBoardSigPoints], dartBoard.doubles[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    for (auto pt : dartBoard.doubles[i]->significantPoints)
    {
      circle(images[DartBoardSigPoints], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
    circle(images[DartBoardSigPoints], dartBoard.triples[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    for (auto pt : dartBoard.triples[i]->significantPoints)
    {
      circle(images[DartBoardSigPoints], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
  }

  // DartBoardSigDoubleTriples
  for (auto i = 0; i < 20; i++)
  {
    //circle(images[DartBoardSigDouble], dartBoard.doubles[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    DartBoard::printText(images[DartBoardSigDoubleTriples], dartBoard.doubles, "D", 1, 2, _greenColor);
    for (auto pt : dartBoard.doubles[i]->significantPoints)
    {
      circle(images[DartBoardSigDoubleTriples], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
  }
  for (auto i = 0; i < 20; i++)
  {
    //circle(images[DartBoardSigDouble], dartBoard.doubles[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    DartBoard::printText(images[DartBoardSigDoubleTriples], dartBoard.triples, "T", 1, 2, _greenColor);
    for (auto pt : dartBoard.triples[i]->significantPoints)
    {
      circle(images[DartBoardSigDoubleTriples], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
  }

  // DartBoardSigBullseye
  circle(images[DartBoardSigBullseye], dartBoard.innerBullseye.meanPoint, 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.innerBullseye.significantPoints[0], 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.innerBullseye.significantPoints[1], 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.outerBullseye.meanPoint, 3, _greenColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.outerBullseye.significantPoints[0], 3, _greenColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.outerBullseye.significantPoints[1], 3, _greenColor, 3);

  // DartBoardCon
  dartBoard.drawBoard(images[DartBoardCon], source.size());
  circle(images[DartBoardCon], dartBoard.outerBullseyeCenter, 5, _whiteColor, 5);

  // DartBoardCorners
  for (auto i = 0; i < 20; i++)
  {
    for (cv::Point point : dartBoard.triples[i]->corners)
    {
      cv::circle(images[DartBoardCorners], point, 2, cv::Scalar(255, 255, 255), 2);
    }
    for (cv::Point point : dartBoard.doubles[i]->corners)
    {
      cv::circle(images[DartBoardCorners], point, 2, cv::Scalar(255, 255, 255), 2);
    }
  }

  // DartBoardSortedMeanCorners
  for (auto i = 0; i < 10; i++)
  {
    //Reds contain meanCorners info, so we'll only iterate through them
    DartArea* dartArea = dartBoard.doubles[i * 2];

    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer1], 5, _redColor, 5);
    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer2], 5, _greenColor, 5);
    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner1], 5, _whiteColor, 5);
    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner2], 5, _blueColor, 5);

    dartArea = dartBoard.triples[i * 2];

    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer1], 5, _redColor, 5);
    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer2], 5, _greenColor, 5);
    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner1], 5, _whiteColor, 5);
    cv::circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner2], 5, _blueColor, 5);
  }

  // DartBoardDrawn
  for(int i = 0; i < 10; i++)
  {
    DartArea* dartAreaDouble = dartBoard.doubles[i * 2];
    
    DartArea* clockwiseNeighbour;
    // If last element, the neighbour is the first
    clockwiseNeighbour = dartBoard.doubles[(i == 9 ? 0 : i * 2 + 2)];

    cv::line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer2], _redColor);
    cv::line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Inner1], _redColor);
    cv::line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);
    cv::line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Inner1], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);

    cv::line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], clockwiseNeighbour->meanCorners[DartArea::Outer2], _greenColor);
    cv::line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Inner1], clockwiseNeighbour->meanCorners[DartArea::Inner2], _greenColor);

    DartArea* dartAreaTriple = dartBoard.triples[i * 2];

    // If last element, the neighbour is the first
    clockwiseNeighbour = dartBoard.triples[(i == 9 ? 0 : i * 2 + 2)];

    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaTriple->meanCorners[DartArea::Outer2], _redColor);
    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaTriple->meanCorners[DartArea::Inner1], _redColor);
    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaTriple->meanCorners[DartArea::Inner2], _redColor);
    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], dartAreaTriple->meanCorners[DartArea::Inner2], _redColor);

    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], clockwiseNeighbour->meanCorners[DartArea::Outer2], _greenColor);
    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], clockwiseNeighbour->meanCorners[DartArea::Inner2], _greenColor);

    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer1], _whiteColor);
    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Outer2], _whiteColor);

    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], dartBoard.outerBullseyeCenter, _whiteColor);
    cv::line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner2], dartBoard.outerBullseyeCenter, _whiteColor);

    cv::circle(images[DartBoardDrawn], dartBoard.outerBullseyeCenter, dartBoard.outerBullseyeMeanRadius, Scalar(0, 100, 0), -1);
    cv::circle(images[DartBoardDrawn], dartBoard.outerBullseyeCenter, dartBoard.innerBullseyeMeanRadius, Scalar(0, 0, 100), -1);
  }

  // DartBoardDrawn2
  for (int i = 0; i < 10; i++)
  {
    DartArea* dartAreaDouble = dartBoard.doubles[i * 2];

    DartArea* clockwiseNeighbour;
    // If last element, the neighbour is the first
    clockwiseNeighbour = dartBoard.doubles[(i == 9 ? 0 : i * 2 + 2)];

    DartArea* dartAreaTriple = dartBoard.triples[i * 2];

    // If last element, the neighbour is the first
    clockwiseNeighbour = dartBoard.triples[(i == 9 ? 0 : i * 2 + 2)];

    cv::line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer1], _whiteColor, 2);
    cv::line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Outer2], _whiteColor, 2);

    cv::line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Inner1], dartBoard.outerBullseyeCenter, _whiteColor, 2);
    cv::line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Inner2], dartBoard.outerBullseyeCenter, _whiteColor, 2);

    cv::circle(images[DartBoardDrawn2], dartBoard.outerBullseyeCenter, dartBoard.outerBullseyeMeanRadius, Scalar(0, 0, 0), -1);
  }
  dartBoard.drawBoard(images[DartBoardDrawn2], source.size());

  std::string name = "Dartboard Progress";
  _win.namedWindowResized(name);
  setMouseCallback(name, mouseCallBack);
  _win.switchableImgs(name, images);
}

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
    FinalBoard,
    FinalBoardOverlay,
    END
  };

  std::list<Mat> testImages;

  testImages.push_back(_win.imreadRel("TestImage.jpg"));
  testImages.push_back(_win.imreadRel("TestImage5.jpg"));
  testImages.push_back(_win.imreadRel("image.jpg"));
  testImages.push_back(_win.imreadRel("image1.jpg"));
  testImages.push_back(_win.imreadRel("image2.jpg"));
  testImages.push_back(_win.imreadRel("image3.jpg"));
  testImages.push_back(_win.imreadRel("image4.jpg"));
  testImages.push_back(_win.imreadRel("image5.jpg"));

  for (const Mat src : testImages)
  {
    bool useHistogramAsLastResort = false;

    prepareBoard:
    std::array<Mat, END> images;

    _win.imgshowResized(windowNameInput, src);

    images[Source] = src;

    if(useHistogramAsLastResort)
    {
      Automation::histogramEqualizationColored(images[Source], images[Histogram]);
    }
    else
    {
      images[Histogram] = images[Source];
    }
    
    Automation::colorFilter(images[Histogram], images[MaskRed], images[FilteredRed], Resources::red_1_histogram, Resources::red_2_histogram);
    Automation::colorFilter(images[Histogram], images[MaskGreen], images[FilteredGreen], Resources::green_histogram);

    Automation::erosion(images[FilteredRed], images[ErosionRed]);
    Automation::erosion(images[FilteredGreen], images[ErosionGreen]);

    const auto contoursRed = Automation::contours(images[ErosionRed], images[DrawingRed], true);
    const auto contoursGreen = Automation::contours(images[ErosionGreen], images[DrawingGreen], true);
    images[DrawingResult] = images[DrawingRed] + images[DrawingGreen];

    const auto dartAreasGreen = DartArea::calculateAreas(contoursGreen);
    const auto dartAreasRed = DartArea::calculateAreas(contoursRed);

    DartBoard dartBoard(dartAreasGreen, dartAreasRed, images[Source]);
    if(!dartBoard.isReady() && !useHistogramAsLastResort)
    {
      std::cout << "Using histograms as last resort!\n";
      useHistogramAsLastResort = true;
      goto prepareBoard;
    }

    //DartBoard::markAreas(image[DrawingResult], dartBoard.doubles, 3, cv::Scalar(0, 0, 255), 3);
    //DartBoard::markAreas(image[DrawingResult], dartBoard.triples, 3, cv::Scalar(0, 0, 255), 3);

    dartBoard.drawBoard(images[FinalBoard], images[Source].size());

    _win.imgshowResized("Histogram", images[Histogram]); 

    _win.imgshowResized("Mask Red", images[MaskRed]);
    _win.imgshowResized("Mask Green", images[MaskGreen]);

    _win.imgshowResized("Filtered Red", images[FilteredRed]);
    _win.imgshowResized("Filtered Green", images[FilteredGreen]);

    _win.imgshowResized("Erosion Red", images[ErosionRed]);
    _win.imgshowResized("Erosion Green", images[ErosionGreen]);

    _win.imgshowResized("Drawing Red", images[DrawingRed]);
    _win.imgshowResized("Drawing Green", images[DrawingGreen]);

    _win.imgshowResized("Contoured Result", images[DrawingResult]);

    images[FinalBoardOverlay] = images[Source] + images[FinalBoard];
    _win.imgshowResized("Final Dartboard", images[FinalBoardOverlay]);

    //bool finalShown = true;
    //do
    //{
    //  _win.imgshowResized("Final Dartboard", finalShown ? image[FinalBoardOverlay] : image[Source]);

    //  finalShown = !finalShown;
    //} while(waitKey(0) == 't');

    drawDartBoardProgressLine(dartBoard, images[Source], images[DrawingResult]);
  }
}

void reset(string windowNameInput)
{
  destroyAllWindows();
  _win.namedWindowResized(windowNameInput);
  imshow(windowNameInput, defInputImage);
}

int main(int argc, char** argv)
{
  string homePath = "/home/pi/Desktop/";

  Mat badQualityImage = _win.imreadRel("TestImage5.jpg");
  Mat hqFinalMask = _win.imreadRel("FinalMask2.jpg");
  Mat hqColorResult = _win.imreadRel("Result.jpg");
  Mat hqContours = _win.imreadRel("Contours.jpg");

  //defInputImage = imread("/home/pi/Desktop/TestImage5.jpg"); // Init inputImage
  defInputImage = _win.imreadRel("TestImage.jpg"); // Init inputImage
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
    Mat result;
    switch (ch)
    {
    //case 'a':
    //  Automation::automatedColorTest(defInputImage, Resources::red_1_default, Resources::red_2_default, Resources::green_default);
    //  waitKey(0);
    //  break;
    case 'b':

      Automation::histogramEqualizationColored(defInputImage, defInputImage);
      colorTest();
      break;
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
      break;
    case 'q':
      return 0;
    default: ;
    }
  }
}
