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

DartBoard* dartboard;
string callBackMatName = "Dartboard Progress";
Mat callBackMatSource;
bool resetCallback = false;

Mat defInputImage;
string windowNameInput = "Input";

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

    auto* area = dartboard->detectHit(cv::Point(x, y));

    if (area != nullptr)
    {
      resetCallback = true;
      //Mat areaImg = Mat::zeros(callBackMatSource.size(), CV_8UC3);
      Mat areaImg = defInputImage.clone();
      area->drawUsingNameColor(areaImg);

      std::stringstream text;
      text << "+" << area->name.getScore();
      putText(areaImg, text.str(), dartboard->titlePoint, 1, 4, _whiteColor, 3);

      _win.imgshowResized(callBackMatName, areaImg);
    }
  }
}

void drawDartBoardProgressLine(DartBoard dartBoard, const Mat& source, const Mat& contours)
{
  dartboard = &dartBoard;

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
  for(auto neighbour : dartBoard.doubles[AREA_20]->neighbours)
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
  circle(images[DartBoardSigBullseye], dartBoard.bullseye.meanPoint, 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.bullseye.significantPoints[0], 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.bullseye.significantPoints[1], 3, _redColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.singleBull.meanPoint, 3, _greenColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.singleBull.significantPoints[0], 3, _greenColor, 3);
  circle(images[DartBoardSigBullseye], dartBoard.singleBull.significantPoints[1], 3, _greenColor, 3);

  // DartBoardCon
  dartBoard.drawBoard(images[DartBoardCon], source.size());
  circle(images[DartBoardCon], dartBoard.singleBullCenter, 5, _whiteColor, 5);

  // DartBoardCorners
  for (auto i = 0; i < 20; i++)
  {
    for (auto point : dartBoard.triples[i]->corners)
    {
      circle(images[DartBoardCorners], point, 2, Scalar(255, 255, 255), 2);
    }
    for (auto point : dartBoard.doubles[i]->corners)
    {
      circle(images[DartBoardCorners], point, 2, Scalar(255, 255, 255), 2);
    }
  }

  // DartBoardSortedMeanCorners
  for (auto i = 0; i < 10; i++)
  {
    //Reds contain meanCorners info, so we'll only iterate through them
    auto dartArea = dartBoard.doubles[i * 2];

    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer1], 5, _redColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer2], 5, _greenColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner1], 5, _whiteColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner2], 5, _blueColor, 5);

    dartArea = dartBoard.triples[i * 2];

    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer1], 5, _redColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Outer2], 5, _greenColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner1], 5, _whiteColor, 5);
    circle(images[DartBoardSortedMeanCorners], dartArea->meanCorners[DartArea::Inner2], 5, _blueColor, 5);
  }

  // DartBoardDrawn
  for(auto i = 0; i < 10; i++)
  {
    auto dartAreaDouble = dartBoard.doubles[i * 2];
    
    DartArea* clockwiseNeighbour;
    // If last element, the neighbour is the first
    clockwiseNeighbour = dartBoard.doubles[(i == 9 ? 0 : i * 2 + 2)];

    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer2], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Inner1], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Inner1], dartAreaDouble->meanCorners[DartArea::Inner2], _redColor);

    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Outer1], clockwiseNeighbour->meanCorners[DartArea::Outer2], _greenColor);
    line(images[DartBoardDrawn], dartAreaDouble->meanCorners[DartArea::Inner1], clockwiseNeighbour->meanCorners[DartArea::Inner2], _greenColor);

    auto dartAreaTriple = dartBoard.triples[i * 2];

    // If last element, the neighbour is the first
    clockwiseNeighbour = dartBoard.triples[(i == 9 ? 0 : i * 2 + 2)];

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaTriple->meanCorners[DartArea::Outer2], _redColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaTriple->meanCorners[DartArea::Inner1], _redColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaTriple->meanCorners[DartArea::Inner2], _redColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], dartAreaTriple->meanCorners[DartArea::Inner2], _redColor);

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], clockwiseNeighbour->meanCorners[DartArea::Outer2], _greenColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], clockwiseNeighbour->meanCorners[DartArea::Inner2], _greenColor);

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer1], _whiteColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Outer2], _whiteColor);

    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner1], dartBoard.singleBullCenter, _whiteColor);
    line(images[DartBoardDrawn], dartAreaTriple->meanCorners[DartArea::Inner2], dartBoard.singleBullCenter, _whiteColor);

    circle(images[DartBoardDrawn], dartBoard.singleBullCenter, dartBoard.singleBullMeanRadius, Scalar(0, 100, 0), -1);
    circle(images[DartBoardDrawn], dartBoard.singleBullCenter, dartBoard.bullseyeMeanRadius, Scalar(0, 0, 100), -1);
  }

  // DartBoardDrawn2
  for (auto i = 0; i < 10; i++)
  {
    auto dartAreaDouble = dartBoard.doubles[i * 2];

    auto dartAreaTriple = dartBoard.triples[i * 2];

    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Outer1], dartAreaDouble->meanCorners[DartArea::Outer1], _whiteColor, 2);
    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Outer2], dartAreaDouble->meanCorners[DartArea::Outer2], _whiteColor, 2);

    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Inner1], dartBoard.singleBullCenter, _whiteColor, 2);
    line(images[DartBoardDrawn2], dartAreaTriple->meanCorners[DartArea::Inner2], dartBoard.singleBullCenter, _whiteColor, 2);

    circle(images[DartBoardDrawn2], dartBoard.singleBullCenter, dartBoard.singleBullMeanRadius, Scalar(0, 0, 0), -1);
  }
  dartBoard.drawBoard(images[DartBoardDrawn2], source.size());

  // DartBoardOverlay
  images[DartBoardOverlay] += images[DartBoardDrawn2];

  std::string name = "Dartboard Progress";
  callBackMatSource = images[DartBoardOverlay].clone();
  //callBackMatSource = images[DartBoardDrawn2].clone();
  _win.namedWindowResized(name);
  setMouseCallback(name, mouseCallBack);
  _win.switchableImgs(name, images);
}

void defaultRun()
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

  for (const auto src : testImages)
  {
    auto useHistogramAsLastResort = false;

  prepareBoard:
    std::array<Mat, END> images;

    _win.imgshowResized(windowNameInput, src);

    images[Source] = src;

    if (useHistogramAsLastResort)
    {
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

    DartBoard dartBoard(dartAreasGreen, dartAreasRed, images[Source]);
    if (!dartBoard.isReady() && !useHistogramAsLastResort)
    {
      std::cout << "Using histograms as last resort!\n";
      useHistogramAsLastResort = true;
      waitKey(0);

      goto prepareBoard;
    }
    else if (!dartBoard.isReady() && useHistogramAsLastResort)
    {
      std::cout << "Error" << std::endl;
      waitKey(0);

      return;
    }

    dartBoard.drawBoard(images[FinalBoard], images[Source].size());

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

void testFunc()
{
  //create Background Subtractor objects
  Ptr<BackgroundSubtractor> pBackSub;
    pBackSub = createBackgroundSubtractorMOG2(500, 16, false);
    //pBackSub = createBackgroundSubtractorKNN();

  auto capture(VideoCapture(0));
  if (!capture.isOpened()) {
    //error in opening the video input
    cerr << "Unable to open Videocapture" << endl;
    return;
  }
  Mat frame, fgMask;
  auto frameDelay = 30;
  auto coolDownStartet = false;
  auto takeFrames = 0;

  auto areaMax = 0;
  vector<vector<Point>> biggestContours;
  Mat biggestContourFrame;
  Mat biggestContourMask;

  RNG rng(time(0)); // RNG with seed of current time
  while (true) 
  {
    capture >> frame;
    if (frame.empty())
      break;
    //update the background model
    pBackSub->apply(frame, fgMask);

    if(frameDelay == 0)
    {
      vector<vector<Point>> contours;
      findContours(fgMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      cvtColor(fgMask, fgMask, COLOR_GRAY2BGR);

      if(coolDownStartet)
      {
        takeFrames--;
      }

      for (auto i = 0; i < contours.size(); i++)
      {
        auto area = contourArea(contours[i]);
        if (!coolDownStartet && area > 15) // If length > 15, start the cooldown
        {
          areaMax = 0;
          coolDownStartet = true;
          takeFrames = 20;
          frameDelay = 5; // Delay 5 frames so the dart doesn't get captured while flying
          break;
        }

        if (takeFrames > 0 && areaMax < area)
        {
          areaMax = area;
          biggestContours = contours;
          biggestContourFrame = frame.clone();
          biggestContourMask = fgMask.clone();
        }
      }
    }
    else
    {
      frameDelay--;
    }

    imshow("FG Mask", fgMask);
    imshow("Frame", frame);

    // get the input from the keyboard
    auto keyboard = waitKey(30);
    if (keyboard == 'q' || keyboard == 27)
      break;

    if(coolDownStartet && takeFrames == 0)
    {
      // Draws the biggest found contours
      auto color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      for (auto i = 0; i < biggestContours.size(); i++)
      {
        if(contourArea(biggestContours[i]) > 15)
        {
          drawContours(biggestContourFrame, biggestContours, i, color);
        }
      }
      imshow("FG Mask", biggestContourMask);
      imshow("Frame", biggestContourFrame);

      waitKey(0);

      frameDelay = 30; // Delay of 30 frames so that the algorithm can get a new background image
      pBackSub->apply(frame, fgMask, 1); // Reset background image
      coolDownStartet = false;
    }
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
  std::cout << "Version: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;

  string homePath = "/home/pi/Desktop/";

  auto badQualityImage = _win.imreadRel("TestImage5.jpg");
  auto hqFinalMask = _win.imreadRel("FinalMask2.jpg");
  auto hqColorResult = _win.imreadRel("Result.jpg");
  auto hqContours = _win.imreadRel("Contours.jpg");

  //defInputImage = imread("/home/pi/Desktop/TestImage5.jpg"); // Init inputImage
  defInputImage = _win.imreadRel("TestImage.jpg"); // Init inputImage
  //defInputImage = imread("/home/pi/Desktop/MaskGreen.jpg"); // Init inputImage

  //time_t timer_begin, timer_end;
  //raspicam::RaspiCam_Cv Camera;

  //cv::Mat image;
  //int nCount = 30;
  ////set camera params
  ////Camera.set(CAP_PROP_FRAME_WIDTH, 2592);
  ////Camera.set(CAP_PROP_FRAME_HEIGHT, 1944);
  //Camera.set(CAP_PROP_FORMAT, CV_8UC3);
  ////Camera.set(CAP_PROP_EXPOSURE, 100);
  //Camera.set(CAP_PROP_FPS, 3);
  //Camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
  //Camera.setExposure(raspicam::RASPICAM_EXPOSURE_AUTO);
  //Camera.setExposureCompensation(5);
  ////Camera.setAWB(raspicam::RASPICAM_AWB_OFF);
  //Camera.setContrast(10);
  //Camera.setSaturation(10);
  ////Camera.setExposureCompensation(raspicam::RASPICAM_EXPOSURE_AUTO);
  //
  ////Open camera
  //cout << "Opening Camera..." << endl;
  //if (!Camera.open()) { cerr << "Error opening the camera" << endl; return -1; }
  ////Start capture
  //cout << "Capturing " << nCount << " frames ...." << endl;
  //time(&timer_begin);
  //int _frameCount = 0;
  //Mat _exposed;
  //double alpha = 1.0 / nCount;
  //float brightFactor = 1.5;

  //for (int i = 0; i < nCount; i++) {
  //  Camera.grab();
  //  Camera.retrieve(image);
  //  if (i % 5 == 0)  cout << "\r captured " << i << " images" << std::flush;

  //  image.convertTo(image, CV_32FC3, 1 / 255.0);
  //  if (_frameCount == 0) {
  //    _exposed = image.clone();
  //    addWeighted(_exposed, 0.0, image, static_cast<double>(alpha * brightFactor), 0.0, _exposed);
  //  }
  //  else {
  //    addWeighted(_exposed, 1.0, image, static_cast<double>(alpha * brightFactor), 0.0, _exposed);
  //  }
  //  _frameCount++;
  //}

  //cout << "Stop camera..." << endl;
  //Camera.release();
  ////show time statistics
  //time(&timer_end); /* get current time; same as: timer = time(NULL)  */
  //double secondsElapsed = difftime(timer_end, timer_begin);
  //cout << secondsElapsed << " seconds for " << nCount << "  frames : FPS = " << (float)((float)(nCount) / secondsElapsed) << endl;
  ////save image 
  //cv::imwrite("raspicam_cv_image.jpg", _exposed);
  //cout << "Image saved at raspicam_cv_image.jpg" << endl;
  ////cvtColor(_exposed, _exposed, COLOR_RGB2BGR);
  ////cvtColor(image, image, COLOR_RGB2BGR);
  //_win.imgshowResized("Exposure Input", _exposed);
  //_win.imgshowResized("Image", image);

  //_exposed.convertTo(defInputImage, CV_8UC3, 255);

  int returnCode = system("raspistill -o input.jpg");

  if(returnCode != 0)
  {
    std::cerr << "Raspistill return code was " << returnCode << ".";
    return -1;
  }

  Mat input = imread("input.jpg");
  _win.imgshowResized("Input", input);

  defInputImage = input;

  //auto cap = VideoCapture(0);
  ////cap.set(CAP_PROP_XI_FRAMERATE, 1);
  ////cap.set(CAP_PROP_FRAME_WIDTH, 2592);
  ////cap.set(CAP_PROP_FRAME_HEIGHT, 1944);
  //cap.set(CAP_PROP_FRAME_WIDTH, 2592);
  //cap.set(CAP_PROP_FRAME_HEIGHT, 1944);
  ////cap.set(CAP_PROP_EXPOSURE, 5000);
  //cap.set(CAP_PROP_AUTO_EXPOSURE, 0.75);
  //cap.read(defInputImage);

  //int exposureFrames = 30;
  //int _frameCount = 0;
  //Mat image;
  //Mat _exposed;
  //double alpha = 1.0 / exposureFrames;
  //int brightFactor = 2;
  //for(int i = 0; i < exposureFrames; i++)
  //{
  //  cap.read(image);
  //  //image.convertTo(image, CV_32FC3, 1 / 255.0);
  //  if (_frameCount == 0) {
  //    _exposed = image.clone();
  //    addWeighted(_exposed, 0.0, image, alpha * brightFactor, 0.0, _exposed);
  //  }
  //  else {
  //    addWeighted(_exposed, 1.0, image, alpha * brightFactor, 0.0, _exposed);
  //  }
  //  _frameCount++;
  //}
  //cvtColor(_exposed, _exposed, COLOR_RGB2BGR);
  //cvtColor(defInputImage, defInputImage, COLOR_RGB2BGR);
  //_win.imgshowResized("Input", defInputImage);
  //_win.imgshowResized("Exposure Input", _exposed);
  //waitKey(0);
  ////cvtColor(defInputImage, defInputImage, COLOR_RGB2BGR); // Needed for VideoCapture b/c OpenCV is dumb

  //defInputImage = _exposed;

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
