#pragma once
#include <string>
#include <opencv2/core/mat.hpp>

#include "Resources.h"

class Testing
{
public:
  Testing(const cv::Mat& input);

  cv::Mat cannyResult;
  void cannyTest();

  cv::Mat colorFilterResult;
  void colorFilter();


  void MOG2Filter();

private:
  cv::Mat input;

  // Canny vars
  cv::Mat cannyGrayInput;
  cv::Mat cannyOutput;
  int cannyThresh = 50;
  int cannyMinPerimeter = 150;
  static void cannyTestCallback(int, void*);
  const std::string cannyContourWindow = "Canny-Edge Detection Contours";
  const std::string cannyOutWindow = "Canny-Edge Detection Output";
  const std::string cannyInputWindow = "Canny-Edge Detection Input";

  // Color Filter vars
  int colorFilterHsvLow[3] = { 0, 0, 0 }; // Default values
  int colorFilterHsvHigh[3] = { 255, 255, 255 }; // Default values
  cv::Mat colorFilterMask;
  cv::Mat colorFilterHsvInput;
  static void colorTestCallback();
  const std::string colorFilterMaskWindow = "Color-Filter Mask";
  const std::string colorFilterOutWindow = "Color-Filter Out";
  const std::string colorFilterInputWindow = "Color-Filter Input";

  // MOG2 Filter vars
  static void MOG2FilterCallback();
  cv::VideoCapture* capture;
  cv::Ptr<cv::BackgroundSubtractorMOG2> pBackSub;

  const std::string windowNameFrame = "Frame";
  cv::Mat cFrame;

  const std::string windowNameMask = "Mask";
  cv::Mat mask;

  const std::string windowNameCanny = "Canny";
  cv::Mat canny;

  int mogValues[3] = { 12, 60, 11 }; // Threshold, Shadow-Threshold, Ellipse-Morph
  int mogValues2[3] = { 50, 100, 11 }; // Canny Edge arg 1, 2, min perimeter
};