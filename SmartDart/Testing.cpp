#include "Testing.h"

#include "HSVTrackbar.h"

Testing* testingInstance;

Testing::Testing(const cv::Mat& input) : input(input.clone())
{
  testingInstance = this;
}

void Testing::cannyTestCallback(int, void*)
{
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  Canny(testingInstance->cannyGrayInput, canny_output, testingInstance->cannyThresh, testingInstance->cannyThresh * 2);

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
    if (perimeter > testingInstance->cannyMinPerimeter)
    {
      std::cout << perimeter << "\n";
      cv::Scalar color = cv::Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));

      drawContours(drawing, contours, static_cast<int>(i), color, 2, cv::LINE_8, hierarchy, 0);
    }
  }

  _win.imgshowResized(testingInstance->cannyContourWindow, drawing);
  _win.imgshowResized(testingInstance->cannyOutWindow, canny_output);
}

void Testing::cannyTest()
{
  _win.imgshowResized(testingInstance->cannyInputWindow, input);

  cvtColor(input, cannyGrayInput, cv::COLOR_BGR2GRAY);
  blur(cannyGrayInput, cannyGrayInput, cv::Size(3, 3));

  const int max_thresh = 255;
  const int maxArea = 500;
  cv::createTrackbar("Canny thresh:", testingInstance->cannyInputWindow, &cannyThresh, max_thresh, cannyTestCallback);
  cv::createTrackbar("MinArea:", testingInstance->cannyInputWindow, &cannyMinPerimeter, maxArea, cannyTestCallback);
  cannyTestCallback(0, 0);

  cv::waitKey(0);
}

void Testing::colorTestCallback()
{
  //// https://alloyui.com/examples/color-picker/hsv.html
  const cv::Scalar min_Color = cv::Scalar(testingInstance->colorFilterHsvLow[0], testingInstance->colorFilterHsvLow[1], testingInstance->colorFilterHsvLow[2]);
  const cv::Scalar max_Color = cv::Scalar(testingInstance->colorFilterHsvHigh[0], testingInstance->colorFilterHsvHigh[1], testingInstance->colorFilterHsvHigh[2]);

  cv::Mat colorFilteredMask;
  inRange(testingInstance->colorFilterHsvInput, min_Color, max_Color, colorFilteredMask);

  cv::Mat result;
  bitwise_and(testingInstance->input, testingInstance->input, result, colorFilteredMask);

  _win.imgshowResized(testingInstance->colorFilterMaskWindow, colorFilteredMask);
  _win.imgshowResized(testingInstance->colorFilterOutWindow, result);
}

void Testing::colorFilter()
{
  _win.imgshowResized(cannyInputWindow, input);

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