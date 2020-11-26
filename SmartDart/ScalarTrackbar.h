#pragma once
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <list>

class ScalarTrackbar
{
protected:
  std::string windowName{ };
  int maxValue;
  int* values[3]{ };
  std::string names[3];
  cv::Mat image;
  cv::TrackbarCallback callback;

public:
  ScalarTrackbar(std::string name, int* value1, int* value2, int* value3,
    std::string name1 = "", std::string name2 = "", std::string name3 = "", int maxValue = 255,
   int width = 300, int height = 300, cv::TrackbarCallback callbackOverride = nullptr);

private:
  static void onChange(int, void*);

  virtual void onChangeInherit(int) = 0;
};