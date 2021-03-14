#include <iostream>
#include "opencv2/opencv.hpp"
#include "HSVTrackbar.h"
#include <thread>
#include <climits>

#include "Automation.h"
#include "DartAreas.h"
#include "DartBoard.h"
#include "WindowHelper.h"
#include "Resources.h"
#include "Testing.h"
#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace std;

DartBoard* dartBoard;

Mat defInputImage;
string windowNameInput = "Input";

void reset(string windowNameInput)
{
  destroyAllWindows();
  _win.namedWindowResized(windowNameInput);
  imshow(windowNameInput, defInputImage);
}

Mat getImage()
{
  const int returnCode = system("raspistill -w 2560 -h 1920 -st -t 1000 -o input.jpg");

  if (returnCode != 0)
  {
    std::cerr << "Raspistill return code was " << returnCode << ".";
    return Mat();
  }

  Mat img = imread("input.jpg");
  //resize(img, img, { 2560, 1920 });
  return img;
}

string callBackMatName = "Dartboard Progress";
Mat callBackMatSource;
bool resetCallback = false;

// Hitbox detection (for now)
void mouseCallBack(int event, int x, int y, int flags, void* userdata)
{
  if (event == EVENT_LBUTTONDOWN)
  {
    if (resetCallback)
    {
      resetCallback = false;
      _win.imgshowResized(callBackMatName, callBackMatSource);
      return;
    }

    auto* area = dartBoard->detectHit(cv::Point(x, y));

    if (area != nullptr)
    {
      resetCallback = true;
      //Mat areaImg = Mat::zeros(callBackMatSource.size(), CV_8UC3);
      Mat areaImg = defInputImage.clone();
      area->drawUsingNameColor(areaImg);

      std::stringstream text;
      text << "+" << area->name.getScore();
      // TODO: Re-add titlepoint
      //putText(areaImg, text.str(), dartBoard->titlePoint, 1, 4, _whiteColor, 3);

      _win.imgshowResized(callBackMatName, areaImg);
    }
  }
}

void drawDartBoardProgressLine(const Mat& source, const Mat& contours)
{
  DartBoard board = *dartBoard;

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
    DartBoardOverlay,
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
    source.clone(),
    Mat::zeros(source.size(),CV_8UC3),
    Mat::zeros(source.size(),CV_8UC3),
    source.clone() };

  // DartAreaSigPoints
  for (auto area : board.greenContours)
  {
    circle(images[DartAreaSigPoints], area.meanPoint, 3, _greenColor, 3);
    for (auto pt : area.significantPoints)
    {
      circle(images[DartAreaSigPoints], pt, 3, _greenColor, 3);
    }
  }
  for (auto area : board.redContours)
  {
    circle(images[DartAreaSigPoints], area.meanPoint, 3, _redColor, 3);
    for (auto pt : area.significantPoints)
    {
      circle(images[DartAreaSigPoints], pt, 3, _redColor, 3);
    }
  }

  // DartBoardNeighbour
  circle(images[DartBoardNeighbour], board.doubles[AREA_20]->meanPoint, 3, _redColor, 3);
  for (auto pt : board.doubles[AREA_20]->significantPoints)
  {
    circle(images[DartBoardNeighbour], pt, 3, _redColor, 3);
  }
  for(auto neighbour : board.doubles[AREA_20]->neighbours)
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
    circle(images[DartBoardSigPoints], board.doubles[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    for (auto pt : board.doubles[i]->significantPoints)
    {
      circle(images[DartBoardSigPoints], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
    circle(images[DartBoardSigPoints], board.triples[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    for (auto pt : board.triples[i]->significantPoints)
    {
      circle(images[DartBoardSigPoints], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
  }

  // DartBoardSigDoubleTriples
  for (auto i = 0; i < 20; i++)
  {
    //circle(images[DartBoardSigDouble], dartBoard.doubles[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    DartBoard::printText(images[DartBoardSigDoubleTriples], board.doubles, "D", 1, 2, _greenColor);
    for (auto pt : board.doubles[i]->significantPoints)
    {
      circle(images[DartBoardSigDoubleTriples], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
  }
  for (auto i = 0; i < 20; i++)
  {
    //circle(images[DartBoardSigDouble], dartBoard.doubles[i]->meanPoint, 3, i % 2 ? _redColor : _greenColor, 3);
    DartBoard::printText(images[DartBoardSigDoubleTriples], board.triples, "T", 1, 2, _greenColor);
    for (auto pt : board.triples[i]->significantPoints)
    {
      circle(images[DartBoardSigDoubleTriples], pt, 3, i % 2 ? _redColor : _greenColor, 3);
    }
  }

  // DartBoardSigBullseye
  circle(images[DartBoardSigBullseye], board.bullseye.meanPoint, 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], board.bullseye.significantPoints[0], 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], board.bullseye.significantPoints[1], 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], board.singleBull.meanPoint, 3, _greenColor, 3);
  circle(images[DartBoardSigBullseye], board.singleBull.significantPoints[0], 3, _greenColor, 3);
  circle(images[DartBoardSigBullseye], board.singleBull.significantPoints[1], 3, _greenColor, 3);

  // DartBoardCon
  board.drawBoard(images[DartBoardCon], source.size());
  circle(images[DartBoardCon], board.singleBullCenter, 5, _whiteColor, 5);

  // DartBoardCorners
  for (auto i = 0; i < 20; i++)
  {
    for (auto point : board.triples[i]->corners)
    {
      circle(images[DartBoardCorners], point, 2, Scalar(255, 255, 255), 2);
    }
    for (auto point : board.doubles[i]->corners)
    {
      circle(images[DartBoardCorners], point, 2, Scalar(255, 255, 255), 2);
    }
  }

  // DartBoardSortedMeanCorners
  for (auto i = 0; i < 10; i++)
  {
    //Reds contain meanCorners info, so we'll only iterate through them
    auto dartArea = board.doubles[i * 2];

    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer1], 5, _redColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer2], 5, _greenColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner1], 5, _whiteColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner2], 5, _blueColor, 5);

    dartArea = board.triples[i * 2];

    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer1], 5, _redColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer2], 5, _greenColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner1], 5, _whiteColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner2], 5, _blueColor, 5);
  }

  // DartBoardDrawn
  for(auto i = 0; i < 10; i++)
  {
    auto dartAreaDouble = board.doubles[i * 2];
    
    DartArea* clockwiseNeighbour;
    // If last element, the neighbour is the first
    clockwiseNeighbour = board.doubles[(i == 9 ? 0 : i * 2 + 2)];

    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer2], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Inner1], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Inner1], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);

    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], clockwiseNeighbour->meanCorners[DartArea::Outer2], _greenColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Inner1], clockwiseNeighbour->meanCorners[DartArea::Inner2], _greenColor);

    auto dartAreaTriple = board.triples[i * 2];

    // If last element, the neighbour is the first
    clockwiseNeighbour = board.triples[(i == 9 ? 0 : i * 2 + 2)];

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaTriple->meanCorners[DartArea::Outer2], _redColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaTriple->meanCorners[DartArea::Inner1], _redColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaTriple->meanCorners[DartArea::Inner2], _redColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], dartAreaTriple->meanCorners[DartArea::Inner2], _redColor);

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], clockwiseNeighbour->meanCorners[DartArea::Outer2], _greenColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], clockwiseNeighbour->meanCorners[DartArea::Inner2], _greenColor);

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer1], _whiteColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Outer2], _whiteColor);

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], board.singleBullCenter, _whiteColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner2], board.singleBullCenter, _whiteColor);

    circle(images[DartBoardDrawn], board.singleBullCenter, board.singleBullMeanRadius, Scalar(0, 100, 0), -1);
    circle(images[DartBoardDrawn], board.singleBullCenter, board.bullseyeMeanRadius, Scalar(0, 0, 100), -1);
  }

  // DartBoardDrawn2
  for (auto i = 0; i < 10; i++)
  {
    auto dartAreaDouble = board.doubles[i * 2];

    auto dartAreaTriple = board.triples[i * 2];

    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer1], _whiteColor, 2);
    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Outer2], _whiteColor, 2);

    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Inner1], board.singleBullCenter, _whiteColor, 2);
    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Inner2], board.singleBullCenter, _whiteColor, 2);

    circle(images[DartBoardDrawn2], board.singleBullCenter, board.singleBullMeanRadius, Scalar(0, 0, 0), -1);
  }
  board.drawBoard(images[DartBoardDrawn2], source.size());

  // DartBoardOverlay
  images[DartBoardOverlay] += images[DartBoardDrawn2];

  std::string name = "Dartboard Progress";
  callBackMatSource = images[DartBoardOverlay].clone();
  //callBackMatSource = images[DartBoardDrawn2].clone();
  _win.namedWindowResized(name);
  setMouseCallback(name, mouseCallBack);
  _win.switchableImgs(name, images);
}

void defaultRun(bool showImgs = true)
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
  
  //Mat inputImage;
  //auto cap = VideoCapture(0);
  //cap.set(CAP_PROP_XI_FRAMERATE, 1);
  //cap.set(CAP_PROP_FRAME_WIDTH, 2592);
  //cap.set(CAP_PROP_FRAME_HEIGHT, 1944);
  //cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
  //cap.read(inputImage);
  //cvtColor(inputImage, inputImage, COLOR_RGB2BGR); // Needed for VideoCapture b/c OpenCV is dumb

  testImages.push_back(defInputImage);
  //testImages.push_back(inputImage);
  testImages.push_back(_win.imreadRel("Img1.jpg"));
  testImages.push_back(_win.imreadRel("TestImage.jpg"));
  testImages.push_back(_win.imreadRel("TestImage5.jpg"));
  testImages.push_back(_win.imreadRel("image.jpg"));
  testImages.push_back(_win.imreadRel("image1.jpg"));
  testImages.push_back(_win.imreadRel("image2.jpg"));
  testImages.push_back(_win.imreadRel("image3.jpg"));
  testImages.push_back(_win.imreadRel("image4.jpg"));
  testImages.push_back(_win.imreadRel("image5.jpg"));

  //for (const auto src : testImages)
  //for(;;)
  //{
  Mat src = getImage();

  auto useHistogramAsLastResort = false;

prepareBoard:
  std::array<Mat, END> images;

  _win.imgshowResized(windowNameInput, src);

  images[Source] = src;

  if (useHistogramAsLastResort)
  {
    waitKey(0);
    Automation::histogramEqualizationColored(images[Source], images[Histogram]);
  }
  else
  {
    images[Histogram] = images[Source];
  }

  if (useHistogramAsLastResort)
  {
    Automation::colorFilter(images[Histogram], images[MaskRed], images[FilteredRed], Resources::red_1_histogram, Resources::red_2_histogram);
    Automation::colorFilter(images[Histogram], images[MaskGreen], images[FilteredGreen], Resources::green_histogram);
  }
  else
  {
    Automation::colorFilter(images[Source], images[MaskRed], images[FilteredRed], Resources::red_1_default, Resources::red_2_default);
    Automation::colorFilter(images[Source], images[MaskGreen], images[FilteredGreen], Resources::green_default);
  }

  Automation::erosion(images[FilteredRed], images[ErosionRed]);
  Automation::erosion(images[FilteredGreen], images[ErosionGreen]);

  const auto contoursRed = Automation::contours(images[ErosionRed], images[DrawingRed], true);
  const auto contoursGreen = Automation::contours(images[ErosionGreen], images[DrawingGreen], true);
  images[DrawingResult] = images[DrawingRed] + images[DrawingGreen];

  const auto dartAreasGreen = DartArea::calculateAreas(contoursGreen);
  const auto dartAreasRed = DartArea::calculateAreas(contoursRed);

  if (showImgs)
  {
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
  }

  dartBoard = new DartBoard(dartAreasGreen, dartAreasRed, images[Source]);
  if (!dartBoard->isReady() && !useHistogramAsLastResort)
  {
    std::cout << "Using histograms as last resort!\n";
    useHistogramAsLastResort = true;

    goto prepareBoard;
  }
  if (!dartBoard->isReady() && useHistogramAsLastResort)
  {
    std::cout << "Error" << std::endl;
    waitKey(0);

    return;
  }

  if (showImgs)
  {
    dartBoard->drawBoard(images[FinalBoard], images[Source].size());

    images[FinalBoardOverlay] = images[Source] + images[FinalBoard];
    _win.imgshowResized("Final Dartboard", images[FinalBoardOverlay]);

    /*bool finalShown = true;
    do
    {
      _win.imgshowResized("Final Dartboard", finalShown ? images[FinalBoardOverlay] : images[Source]);

      finalShown = !finalShown;
    } while(waitKey(0) == 't');
    */
    drawDartBoardProgressLine(images[Source], images[DrawingResult]);
  }
  //}
}

Mat cFrame;

void getter()
{
  VideoCapture capture(0);
  capture.set(CAP_PROP_FRAME_WIDTH, 2560);
  capture.set(CAP_PROP_FRAME_HEIGHT, 1920);

  if (!capture.isOpened()) {
    //error in opening the video input
    cerr << "Unable to open Videocapture" << endl;
    return;
  }

  for(;;)
  {
    capture.read(cFrame);
  }
}

void testFunc()
{
  defaultRun(false);

  std::thread thread1(getter);

  int minLength = arcLength(dartBoard->bullseye.contour, true) / 2;
  int maxLength = arcLength(dartBoard->singles[0]->contour, true);
  
  //create Background Subtractor objects
  Ptr<BackgroundSubtractor> pBackSub;

  auto test = createBackgroundSubtractorMOG2(500, 16, false);
  pBackSub = test;
  //pBackSub = createBackgroundSubtractorKNN();

  Mat fgMask;

  vector<vector<Point>> biggestContour;
  Mat biggestContourFrame;
  Mat biggestContourMask;

  int i = 0;

  std::chrono::time_point<std::chrono::steady_clock> start, end;
  chrono::duration<long long, ratio<1, 1000>> diff = chrono::duration<long long, ratio<1, 1000>>(0);

  _win.namedWindowResized("Dartboard");
  _win.namedWindowResized("Frame");
  _win.namedWindowResized("fgMask");
  _win.namedWindowResized("Contour");

  Mat dartboardImg = Mat::zeros(defInputImage.size(), CV_8UC3);
  dartBoard->drawBoard(dartboardImg, defInputImage.size());
  rectangle(dartboardImg, dartBoard->rect, _greenColor, 3);

  while (cFrame.empty()) // Wait for first frame;
  { }

  cvtColor(cFrame, cFrame, COLOR_RGB2BGR);
  imshow("Dartboard", dartboardImg);
  //resize(dartboardImg, dartboardImg, frame.size(), 0.5625, 0.75);

  vector<vector<Point>> contours;
  std::vector<Vec4i> hierarchy;
  Mat contourImg = cFrame.clone();

  Mat frame = cFrame(dartBoard->rect);

  bool startFrameCooldown = false;
  int frameCoolDown = 0;
  Mat calculatedFrame = Mat::zeros(frame.size(), CV_8UC1);
  std::vector<Mat> frames;

  while (true)
  {
    start = std::chrono::steady_clock::now();

    frame = cFrame(dartBoard->rect);

    //update the background model
    pBackSub->apply(frame, fgMask);

    // get the input from the keyboard
    auto keyboard = waitKey(10);
    if (keyboard == 'q' || keyboard == 27)
      break;

    findContours(fgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    end = std::chrono::steady_clock::now();

    i++;
    diff += std::chrono::duration_cast<
      std::chrono::milliseconds>(end - start);
    //if(diff > chrono::duration<long long, ratio<1, 1000>>(1000))
    //{
      cout << diff.count() << "ms " << static_cast<float>(i / (diff.count() / 1000.0f)) << "fps\r" << std::flush;
      i = 0;
      diff = chrono::duration<long long, ratio<1, 1000>>(0);

      imshow("Frame", cFrame + dartboardImg);
      imshow("fgMask", fgMask);

      contourImg = Mat::zeros(frame.size(), CV_8UC3);
      long xSum = 0;
      long ySum = 0;
      long takenPoints = 0;
      for (int i = 0; i < contours.size(); i++)
      {
        if(arcLength(contours[i], true) > 5)
        {
          drawContours(contourImg, contours, i, _greenColor, -1);
        }
      }

      cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
      cv::morphologyEx(contourImg, contourImg, cv::MORPH_CLOSE, structuringElement);

      Mat contourMask;
      cvtColor(contourImg, contourMask, COLOR_BGR2GRAY);
      contours.clear();
      findContours(contourMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      Mat img = frame.clone();
      for (int i = 0; i < contours.size(); i++)
      {
        int lgth = arcLength(contours[i], true);
        if(lgth > minLength && lgth < maxLength)
        {
          drawContours(img, contours, i, _whiteColor, -1);
        }
      }
      imshow("Contour", img);
    //}
  }
}

int main(int argc, char** argv)
{
  std::cout << "Version: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;

  string homePath = "/home/pi/Desktop/";

  auto badQualityImage = _win.imreadRel("TestImage5.jpg");
  auto hqFinalMask = _win.imreadRel("FinalMask2.jpg");
  auto hqColorResult = _win.imreadRel("Result.jpg");
  auto hqContours = _win.imreadRel("Contours.jpg");

  defInputImage = getImage();
  if (defInputImage.empty())
  {
    return -1;
  }

  _win.imgshowResized(windowNameInput, defInputImage);


  if(defInputImage.data == nullptr)
  {
    std::cout << "Couldn't find image, closing!";
    return -1;
  }

  for (;;)
  {
    reset(windowNameInput);

    auto ch = waitKey(0);
    Mat result;
    switch (ch)
    {
    case 'a':
      defaultRun();
      break;
    case 'b':
    {
      Testing testing(defInputImage);
      testing.colorFilter();
      break;
    }
    case 'c':
    {
      Mat out;
      Automation::histogramEqualizationColored(defInputImage, out);
      Testing testing(out);
      testing.colorFilter();
      break;
    }
    case 'd':
      {
        Testing testing(defInputImage);
        testing.cannyTest();
      }
      break;
    case 't':
      testFunc();
      waitKey(0);
      break;
    case 'q':
      return 0;
    default: ;
    }
  }
}
