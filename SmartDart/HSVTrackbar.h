#pragma once
#include "ScalarTrackbar.h"

class HSVTrackbar : ScalarTrackbar
{
public:
  HSVTrackbar(std::string name, int values[3], int width = 300, int height = 300);

  void setCallback(void(*customCallback)());

  void onChangeInherit(int) override;

private:
  void(*customCallback)() = nullptr;
  cv::Mat hsvImage;
};
