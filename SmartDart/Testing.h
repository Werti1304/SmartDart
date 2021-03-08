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

private:
  cv::Mat input;

  cv::Mat cannyGrayInput;
  cv::Mat cannyOutput;
  int cannyThresh = 50;
  int cannyMinPerimeter = 150;
  static void cannyTestCallback(int, void*);
  const std::string cannyContourWindow = "Canny-Edge Detection Contours";
  const std::string cannyOutWindow = "Canny-Edge Detection Output";
  const std::string cannyInputWindow = "Canny-Edge Detection Input";

  int colorFilterHsvLow[3] = { 0, 0, 0 }; // Default values
  int colorFilterHsvHigh[3] = { 255, 255, 255 }; // Default values
  cv::Mat colorFilterMask;
  cv::Mat colorFilterHsvInput;
  static void colorTestCallback();
  const std::string colorFilterMaskWindow = "Color-Filter Mask";
  const std::string colorFilterOutWindow = "Color-Filter Out";
  const std::string colorFilterInputWindow = "Color-Filter Input";
};