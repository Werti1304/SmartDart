#include <atomic>
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
#include "PointHelper.h"

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

  // contoursSurround for red, because our dart surround is red and need to be filtered out
  const auto contoursRed = Automation::contoursSurround(images[ErosionRed], images[DrawingRed], true);
  //const auto contoursRed = Automation::contours(images[ErosionRed], images[DrawingRed], true);
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
VideoCapture* capture;
std::atomic<bool> done(false);

void readImg()
{
  capture->read(cFrame);
  done = true;
}

void testFunc()
{
  defaultRun(false);

  capture = new VideoCapture(0);
  capture->set(CAP_PROP_FRAME_WIDTH, 2560);
  capture->set(CAP_PROP_FRAME_HEIGHT, 1920);

  if (!capture->isOpened()) {
    //error in opening the video input
    cerr << "Unable to open Videocapture" << endl;
    return;
  }

  readImg();

   std::cout << "Dartboard Rect: " << dartBoard->rect.x << " x " << dartBoard->rect.y << std::endl;

   const unsigned rectWidth = 900;
   const double scaleFactor = static_cast<double>(dartBoard->rect.width) / rectWidth;
   int rectHeight = dartBoard->rect.height / scaleFactor;
   std::cout << "Frame: " << rectWidth << " x " << rectHeight << " with factor " << scaleFactor << std::endl;

  const int minLength = arcLength(dartBoard->bullseye.contour, true) / 2.5;
  const int maxLength = arcLength(dartBoard->singles[0]->contour, true);
  const int minArea = contourArea(dartBoard->bullseye.contour) / 2.5;

  const int morphSizePar = 2 * (rectWidth / 250) + 1;
  const Size morphSize = { morphSizePar, morphSizePar };
  std::cout << "Morphsize: " << morphSize << std::endl;

  const double minDist = static_cast<double>(rectWidth) / 30;
  std::cout << "minDist: " << minDist << std::endl;

  const int maxDartPartDist = dartBoard->rect.width / 4;

  Point scoreDrawPoint = { dartBoard->rect.x + dartBoard->rect.width / 2, dartBoard->rect.y + dartBoard->rect.height };
  Point scoreDrawPoint2 = { dartBoard->rect.x + dartBoard->rect.width / 2, dartBoard->rect.y + 1.1 * dartBoard->rect.height };
  Point scoreDrawPoint3 = { dartBoard->rect.x + dartBoard->rect.width / 2, dartBoard->rect.y + 1.2 * dartBoard->rect.height };
  Point textDrawPoint = { dartBoard->rect.x, dartBoard->rect.y };
  std::cout << "ScoreDrawPoint: " << scoreDrawPoint.x << "/" << scoreDrawPoint.y << std::endl;
  std::cout << "TextDrawPoint: " << textDrawPoint.x << "/" << textDrawPoint.y << std::endl;

  //create Background Subtractor objects
  Ptr<BackgroundSubtractor> pBackSub;

  auto test = createBackgroundSubtractorMOG2(300, 12, true);
  test->setShadowValue(100);
  //std::cout << test->getShadowThreshold() << std::endl;
  test->setShadowThreshold(.75);
  pBackSub = test;
  //pBackSub = createBackgroundSubtractorKNN();

  Mat fgMask;

  Mat biggestContourFrame;
  Mat biggestContourMask;

  int i = 0;

  std::chrono::time_point<std::chrono::steady_clock> start, end;
  chrono::duration<long long, ratio<1, 1000>> diff = chrono::duration<long long, ratio<1, 1000>>(0);

  _win.namedWindowResized("Dartboard");
  _win.namedWindowResized("Frame");
  _win.namedWindowResized("fgMask");
  _win.namedWindowResized("Contour");
  _win.namedWindowResized("Probable Impact");
  _win.namedWindowResized("Balanced Impact");
  _win.namedWindowResized("Convex Impact");
  _win.namedWindowResized("Final");

  Mat dartboardImg = Mat::zeros(defInputImage.size(), CV_8UC3);
  dartBoard->drawBoard(dartboardImg, defInputImage.size());
  rectangle(dartboardImg, dartBoard->rect, _greenColor, 3);

  while (!done) // Wait for first frame;
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  cvtColor(cFrame, cFrame, COLOR_RGB2BGR);
  imshow("Dartboard", dartboardImg);

  vector<vector<Point>> contours;
  std::vector<Vec4i> hierarchy;
  Mat contourImg = cFrame.clone();

  Mat frame = cFrame(dartBoard->rect);

  Mat finalImg = cFrame.clone();
  putText(finalImg, "Starting..", textDrawPoint, 0, 2, _redColor, 2);
  imshow("Final", finalImg);

  std::vector<int> probableContoursIdx;

  std::vector<Mat> frames;

  long maxWhitePixels = (rectWidth * rectHeight) / 10;

  bool waitTillReset = true;
  int frameCountdown = -1;

  while (true)
  {
    start = std::chrono::steady_clock::now();

    frame = cFrame(dartBoard->rect);
    resize(frame, frame, { rectWidth, rectHeight }, 0, 0, INTER_LANCZOS4);

    finalImg = cFrame.clone();

    while (!done) // Wait for new frame;
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    done = false;
    thread t1(readImg);
    t1.detach();

    //update the background model
    pBackSub->apply(frame, fgMask, frameCountdown >= 0 ? 0 : -1);

    imshow("fgMask", fgMask);

    // get the input from the keyboard
    auto keyboard = waitKey(10);
    if (keyboard == 'q' || keyboard == 27)
      break;

    threshold(fgMask, fgMask, 254, 255, THRESH_BINARY);

    GaussianBlur(fgMask, fgMask, { 11, 11 }, 0);
    Canny(fgMask, fgMask, 70, 2 * 70);
    _win.imgshowResized("fgmask2", fgMask);

    //findContours(fgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    end = std::chrono::steady_clock::now();

    i++;
    diff += std::chrono::duration_cast<
      std::chrono::milliseconds>(end - start);
    //if(diff > chrono::duration<long long, ratio<1, 1000>>(1000))
    //{
    cout << diff.count() << "ms " << static_cast<float>(i / (diff.count() / 1000.0f)) << "fps\r" << std::flush;
    i = 0;
    diff = chrono::duration<long long, ratio<1, 1000>>(0);

    //imshow("Frame", cFrame + dartboardImg);
    //imshow("fgMask", fgMask);
    //contourImg = Mat::zeros(frame.size(), CV_8UC3);
    // Draw the bigger contours to dilate them
    
    //for (int i = 0; i < contours.size(); i++)
    //{
    //  if (arcLength(contours[i], true) > minLength)
    //  {
    //    drawContours(contourImg, contours, i, _greenColor, -1);
    //  }
    //}

    //cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, morphSize);
    //cv::morphologyEx(contourImg, contourImg, cv::MORPH_CLOSE, structuringElement);

    //Mat contourMask;
    //cvtColor(contourImg, contourMask, COLOR_BGR2GRAY);
    //contours.clear();
    //findContours(contourMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, morphSize);
    cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, structuringElement);

    findContours(fgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    _win.imgshowResized("fgmask3", fgMask);

    Mat img = frame.clone();

    int biggestContourIdx = -1;
    int biggestArcLength = 0;
    int probableSize = 0;
    probableContoursIdx.clear();

    if (contours.size() > 0)
    {
      for (int i = 0; i < contours.size(); i++)
      {
        int lgth = arcLength(contours[i], true);
        int area = contourArea(contours[i]);
        if (lgth > minLength && lgth < maxLength && area > minArea)
        {
          drawContours(img, contours, i, _redColor, 3);
          probableSize++;
          probableContoursIdx.push_back(i);

          if (lgth > biggestArcLength)
          {
            biggestArcLength = lgth;
            biggestContourIdx = i;
          }
        }
      }

      if (!waitTillReset && biggestContourIdx != -1)
      {
        auto biggestContour = contours.at(biggestContourIdx);

        drawContours(img, contours, biggestContourIdx, _whiteColor, 3);

        Point medianPt = PointHelper::getMassCenter(biggestContour);
        circle(img, medianPt, 5, _greenColor, 3);

        // If we have more than one probable contour...
        if (probableSize > 1)
        {
          for (auto idx : probableContoursIdx)
          {
            auto tmpContour = contours.at(idx);

            // the biggest contour has to be skipped so that we don't add it to itself
            if(tmpContour[0].x == biggestContour[0].x && tmpContour[0].y == biggestContour[0].y)
            {
              continue;
            }

            Moments tmpM = moments(tmpContour);
            Point medianPtTmp(tmpM.m10 / tmpM.m00, tmpM.m01 / tmpM.m00);
            // ...near the biggest contour...
            if (PointHelper::getDistance(medianPt, medianPtTmp) < maxDartPartDist)
            {
              //...ignore it for now and want to add it to the biggest contour later
              probableSize--;
              drawContours(img, contours, idx, _greenColor, 3);
              //biggestContour.insert(biggestContour.end(), tmpContour.begin(), tmpContour.end()); THIS IS NOT OKAY (wrong format)
            }
          }
        }

        // Convex hull impactpoint START
        vector<vector<Point>> hull ( 1 );
        convexHull(biggestContour, hull[0]);
        //drawContours(img, hull, 0, _blueColor, 1);
        Vec4f vectorLineConvex;
        fitLine(hull[0], vectorLineConvex, DIST_L2, 0, 0.01, 0.01);

        // Collects points that are on or in dart contour
        std::vector<Point> convexPts;
        // Update median point (Contour not 100% reliable)
        medianPt = PointHelper::getMassCenter(hull[0]);

        Vec4f vectorLine;
        fitLine(biggestContour, vectorLine, DIST_L2, 0, 0.01, 0.01);

        Point point0;  
        point0.x = vectorLine[2];//point on the line
        point0.y = vectorLine[3];
        double k = vectorLine[1] / vectorLine[0]; //slope

        // Collects points that are on or in dart contour
        std::vector<Point> pts;
        for (int i = 0; i < img.cols; i += 1)
        {
          Point tmp(i, k * (i - point0.x) + point0.y);

          int dist = pointPolygonTest(hull[0], tmp, false);
          //std::cout << "Dist: " << dist << std::endl;
          if (dist >= 0)
          {
            convexPts.push_back(tmp);
          }
          if (dist == 1)
          {
            pts.push_back(tmp);
          }
        }

        Point convexImpactPoint = PointHelper::findPoints(convexPts, std::array<Point, 1>{ medianPt }, false)[0];
        Point oppositeToConvexImpactPoint = PointHelper::findPoints(convexPts, std::array<Point, 1>{ convexImpactPoint }, false)[0];
        Point convexScaledPoint = dartBoard->rect.tl() + scaleFactor * convexImpactPoint;
        auto convexArea = dartBoard->detectHit(convexScaledPoint);

        // Calculate (probable) impactPoint and minDistance
        Point probableImpactPoint = PointHelper::findPoints(pts, std::array<Point, 1>{ medianPt }, false)[0];
        Point oppositeToProbableImpactPoint = PointHelper::findPoints(pts, std::array<Point, 1>{ probableImpactPoint }, false)[0];

        int dartLength = PointHelper::getDistance(probableImpactPoint, oppositeToProbableImpactPoint);
        double minDistFactor = static_cast<double>(dartLength) / rectWidth;

        Point probableScaledPoint = dartBoard->rect.tl() + scaleFactor * probableImpactPoint;
        auto probableArea = dartBoard->detectHit(probableScaledPoint);

        // Calculate real impactPoint
        pts.clear();
        for (int i = 0; i < img.cols; i += 1)
        {
          Point tmp(i, k * (i - point0.x) + point0.y);

          double dist = pointPolygonTest(hull[0], tmp, true);
          //std::cout << "Dist: " << dist << std::endl;
          if (dist >= -(minDistFactor * minDist))
          {
            pts.push_back(tmp);
          }
        }
        Point balancedImpactPoint = PointHelper::findPoints(pts, std::array<Point, 1>{ medianPt }, false)[0];
        Point oppositeToBalancedImpactPoint = PointHelper::findPoints(pts, std::array<Point, 1>{ balancedImpactPoint }, false)[0];

        Point scaledPoint = dartBoard->rect.tl() + scaleFactor * balancedImpactPoint;
        auto balancedArea = dartBoard->detectHit(scaledPoint);

        const Point drawPoint = { balancedImpactPoint.x, balancedImpactPoint.y + dartBoard->rect.y / 5 };
        if(drawPoint.y > dartBoard->rect.y)
        {
          drawPoint.y - dartBoard->rect.y / 2.5;
        }

        string balancedScore;
        if (balancedArea == nullptr)
        {
          balancedScore = "Come back in 100 years";
        }
        else
        {
          balancedScore = "+";
          balancedScore += to_string(balancedArea->name.getScore());
        }

        string probableScore;
        if (probableArea == nullptr)
        {
          probableScore = "Come back in 100 years";
        }
        else
        {
          probableScore = "+";
          probableScore += to_string(probableArea->name.getScore());
        }

        string convexScore = "";
        if (convexArea == nullptr)
        {
          convexScore = "Come back in 100 years";
        }
        else
        {
          convexScore = "+";
          convexScore += to_string(convexArea->name.getScore());
        }

        imshow("Contour", img);

        if (probableSize == 1)
        {
          if (frameCountdown == -1)
          {
            // Wait 2 frames till you show the image
            frameCountdown = 2;
          }
          else if (frameCountdown == 0)
          {
            waitTillReset = true;

            Mat imgProbable = frame.clone();
            Mat imgBalanced = frame.clone();
            Mat imgConvex = frame.clone();

            putText(finalImg, balancedScore, scoreDrawPoint, 0, 2, _greenColor, 3);
            if(balancedArea != probableArea)
            {
              putText(finalImg, probableScore, scoreDrawPoint2, 0, 2, _blueColor, 3);
            }
            if (convexArea != balancedArea && convexArea != probableArea)
            {
              putText(finalImg, convexScore, scoreDrawPoint3, 0, 2, _redColor, 3);
            }

            circle(finalImg, scaledPoint, 3, _blueColor, -1);
            imshow("Final", finalImg);
            imshow("Frame", cFrame + dartboardImg);

            std::cout << std::endl << "NEW DART" << std::endl;
            std::cout << "DartLength: " << dartLength << std::endl;
            std::cout << "MinDistFactor for dart: " << minDistFactor << std::endl;
            std::cout << "MinDist for dart: " << minDistFactor * minDist << std::endl;

            // Draw probable
            drawContours(imgConvex, hull, 0, _blueColor, 1);
            line(imgProbable, probableImpactPoint, oppositeToProbableImpactPoint, _whiteColor, 2);
            circle(imgProbable, probableImpactPoint, 5, _blueColor, 2);
            circle(imgProbable, oppositeToProbableImpactPoint, 3, _blueColor, -1);
            putText(imgProbable, probableScore, drawPoint, 0, 1, _blueColor);
            imshow("Probable Impact", imgProbable);

            // Draw balanced
            drawContours(imgConvex, hull, 0, _greenColor, 1);
            line(imgBalanced, balancedImpactPoint, oppositeToBalancedImpactPoint, _whiteColor, 2);
            circle(imgBalanced, balancedImpactPoint, 5, _greenColor, 2);
            circle(imgBalanced, oppositeToBalancedImpactPoint, 3, _greenColor, -1);
            putText(imgBalanced, balancedScore, drawPoint, 0, 1, _greenColor);
            imshow("Balanced Impact", imgBalanced);

            // Draw convex
            drawContours(imgConvex, hull, 0, _redColor, 1);
            line(imgConvex, convexImpactPoint, oppositeToConvexImpactPoint, _whiteColor, 2);
            circle(imgConvex, convexImpactPoint, 5, _redColor, 2);
            circle(imgConvex, oppositeToConvexImpactPoint, 3, _redColor, -1);
            putText(imgConvex, convexScore, drawPoint, 0, 1, _redColor);
            imshow("Convex Impact", imgConvex);

            // Update background subtraction while nothing is pressed to speed things up
            while (waitKey(50) != ' ')
            {
              readImg();
              frame = cFrame(dartBoard->rect);

              //update the background model
              pBackSub->apply(frame, fgMask);
            }
          }
          else
          {
            frameCountdown--;
          }
        }
      }
    }

    // Every time no matching contours are found
    if (probableSize == 0)
    {
      if(waitTillReset)
      {
        waitTillReset = false; // Stop waiting for the contours to go away
        imshow("Contour", frame);
        imshow("Final", finalImg);
      }
      frameCountdown = -1; // Reset the frame cooldown to inactive
    }
    else if(waitTillReset)
    {
      putText(finalImg, "Please hold the line", textDrawPoint, 0, 2, _redColor, 2);
      imshow("Final", finalImg);
    }
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
    case 'e':
    {
      Testing testing(defInputImage);
      testing.MOG2Filter();
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
