#include "ScalarTrackbar.h"
#include "HSVTrackbar.h"

ScalarTrackbar::ScalarTrackbar(std::string name, int* value1, int* value2, int* value3, std::string name1,
  std::string name2, std::string name3, int maxValue, int width, int height, cv::TrackbarCallback callbackOverride)
{
  this->windowName = name;
  this->maxValue = maxValue;
  this->values[0] = value1;
  this->values[1] = value2;
  this->values[2] = value3;
  this->names[0] = name1;
  this->names[1] = name2;
  this->names[2] = name3;

  if(callbackOverride)
  {
    this->callback = callbackOverride;
  }
  else
  {
    this->callback = onChange;
  }

  // initialize Image
  this->image = cv::Mat3b(width, height, cv::Vec3b(0, 0, 0));

  // initialize trackbars
  // https://docs.opencv.org/3.4/da/d6a/tutorial_trackbar.html
  cv::namedWindow(windowName);
  cv::createTrackbar(names[0], windowName, values[0], maxValue, callback, this);
  cv::createTrackbar(names[1], windowName, values[1], maxValue, callback, this);
  cv::createTrackbar(names[2], windowName, values[2], maxValue, callback, this);
}

void ScalarTrackbar::onChange(int value, void* ScalarTrackbarPtr)
{
  auto hsvTrackbar = static_cast<HSVTrackbar*>(ScalarTrackbarPtr);
  hsvTrackbar->onChangeInherit(value);
}