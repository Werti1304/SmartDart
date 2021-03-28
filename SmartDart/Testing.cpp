#include "Testing.h"

#include "HSVTrackbar.h"
#include "Automation.h"
#include "GeneralTrackbar.h"

Testing* testInst;

Testing::Testing(const cv::Mat& input) : input(input.clone())
{
  testInst = this;
}

void Testing::cannyTestCallback(int, void*)
{
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  Canny(testInst->cannyGrayInput, canny_output, testInst->cannyThresh, testInst->cannyThresh * 2);

  findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat drawing = cv::Mat::zeros(canny_output.size(), CV_8UC3);
  cv::RNG rng(time(0)); // RNG with seed of current time

  for (size_t i = 0; i < contours.size(); i++)
  {
    // Perimeter
    std::vector<cv::Point_<int>> approx_curve;
    auto currentContour = contours.at(i);
    const auto epsilon = 0.1 * arcLength(currentContour, true);
    approxPolyDP(currentContour, approx_curve, epsilon, true);
    const auto perimeter = arcLength(approx_curve, true);

    //if (contourArea(contours.at(i)) > minArea)
    if (perimeter > testInst->cannyMinPerimeter)
    {
      std::cout << perimeter << "\n";
      cv::Scalar color = cv::Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));

      drawContours(drawing, contours, static_cast<int>(i), color, 2, cv::LINE_8, hierarchy, 0);
    }
  }

  _win.imgshowResized(testInst->cannyContourWindow, drawing);
  _win.imgshowResized(testInst->cannyOutWindow, canny_output);
}

void Testing::cannyTest()
{
  _win.imgshowResized(testInst->cannyInputWindow, input);

  cvtColor(input, cannyGrayInput, cv::COLOR_BGR2GRAY);
  blur(cannyGrayInput, cannyGrayInput, cv::Size(3, 3));

  const int max_thresh = 255;
  const int maxArea = 500;
  cv::createTrackbar("Canny thresh:", testInst->cannyInputWindow, &cannyThresh, max_thresh, cannyTestCallback);
  cv::createTrackbar("MinArea:", testInst->cannyInputWindow, &cannyMinPerimeter, maxArea, cannyTestCallback);
  cannyTestCallback(0, 0);

  cv::waitKey(0);
}

void Testing::colorTestCallback()
{
  //// https://alloyui.com/examples/color-picker/hsv.html
  const cv::Scalar min_Color = cv::Scalar(testInst->colorFilterHsvLow[0], testInst->colorFilterHsvLow[1], testInst->colorFilterHsvLow[2]);
  const cv::Scalar max_Color = cv::Scalar(testInst->colorFilterHsvHigh[0], testInst->colorFilterHsvHigh[1], testInst->colorFilterHsvHigh[2]);

  cv::Mat colorFilteredMask;
  inRange(testInst->colorFilterHsvInput, min_Color, max_Color, colorFilteredMask);

  cv::Mat result;
  bitwise_and(testInst->input, testInst->input, result, colorFilteredMask);

  _win.imgshowResized(testInst->colorFilterMaskWindow, colorFilteredMask);
  _win.imgshowResized(testInst->colorFilterOutWindow, result);
}

void Testing::colorFilter()
{
  cvtColor(input, colorFilterHsvInput, cv::COLOR_BGR2HSV); // Init inputImageHsv

  // Create trackbar for low bar of colors
  HSVTrackbar trackbarLow("HSV-Trackbar Low", colorFilterHsvLow);

  // Create trackbar for high bar of colors
  HSVTrackbar trackbarHigh("HSV-Trackbar High", colorFilterHsvHigh);

  // First run to init the images
  colorTestCallback();

  for (;;)
  {
    const auto ch = cv::waitKey(0);
    if (ch == 'q')
    {
      return;
    }
    if (ch == 'c')
    {
      trackbarLow.setCallback(colorTestCallback);
      trackbarHigh.setCallback(colorTestCallback);
    }
    else if (ch == 'r')
    {
      trackbarLow.setCallback(nullptr);
      trackbarHigh.setCallback(nullptr);
    }
    else
    {
      colorTestCallback();
    }
  }
}

void Testing::MOG2FilterCallback()
{
  testInst->capture->read(testInst->cFrame);

  if(testInst->mogValues[2] % 2 == 1 && testInst->mogValues[2] > 0)
  {
    GaussianBlur(testInst->cFrame, testInst->cFrame, { testInst->mogValues[2], testInst->mogValues[2] }, 2, 2);
  }

  //cvtColor(testInst->cFrame, testInst->cFrame, COLOR_BGR2HSV);

  //Automation::histogramEqualizationColored(testInst->cFrame, testInst->cFrame);

  testInst->pBackSub->apply(testInst->cFrame, testInst->mask);

  //cvtColor(testInst->cFrame, testInst->cFrame, COLOR_HSV2BGR);

  //threshold(testInst->mask, testInst->canny, 254, 255, THRESH_BINARY);
  //cv::Canny(testInst->mask, testInst->canny, testInst->mogValues2[0], testInst->mogValues2[1]);
}

void Testing::MOG2Filter()
{
  _win.namedWindowResized(windowNameFrame);
  _win.namedWindowResized(windowNameMask);
  _win.namedWindowResized(windowNameCanny);

  capture = new VideoCapture(0);
  capture->set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  capture->set(cv::CAP_PROP_FRAME_HEIGHT, 720);

  if (!capture->isOpened()) {
    //error in opening the video input
    std::cerr << "Unable to open Videocapture" << std::endl;
    return;
  }

  pBackSub = cv::createBackgroundSubtractorMOG2(500, 12, true);
  pBackSub->setShadowValue(100); // Graytone of shadow
  pBackSub->setShadowThreshold(.75); // Higher means less shadows

    // Create trackbar for low bar of colors
  GeneralTrackbar trackbarMOG("MOG2 Trackbar", mogValues, "Threshold", "ShadowThresh", "MorphSize");

  // Create trackbar for high bar of colors
  GeneralTrackbar trackbarMOG2("Canny Trackbar", mogValues2, "Threshold", "Threshold2", "Min Perimeter");

  for(;;)
  {
    const int varThreshold = pBackSub->getVarThreshold();
    const double shadowThreshold = pBackSub->getShadowThreshold();
    const double newShadowThreshold = static_cast<double>(mogValues[1]) / 100;

    if(mogValues[0] != varThreshold)
    {
      pBackSub->setVarThreshold(mogValues[0]);
      std::cout << "Changed varThreshold from " << varThreshold << " to " << mogValues[0] << std::endl;
    }
    if(round(newShadowThreshold * pow(10, 2)) != round(shadowThreshold * pow(10, 2)))
    {
      pBackSub->setShadowThreshold(newShadowThreshold);
      std::cout << "Changed shadowThreshold from " << shadowThreshold << " to " << newShadowThreshold << std::endl;
    }

    MOG2FilterCallback();

    cv::imshow(windowNameFrame, cFrame);

    cv::imshow(windowNameMask, mask);

    auto c = waitKey(10);

    if(c == ' ')
    {
      Mat test;
      threshold(mask, test, 254, 255, THRESH_BINARY);
      _win.imgshowResized("Test", test);
      //cv::imshow(windowNameCanny, canny);
      waitKey(0);
    }
    else if(c == 'q')
    {
      return;
    }
  }
}
