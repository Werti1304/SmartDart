#pragma once
#include "ScalarTrackbar.h"

class GeneralTrackbar : ScalarTrackbar
{
public:
  GeneralTrackbar(std::string name, int values[3], 
    std::string firstValName, std::string secondValName, std::string thirdValName, 
    int width = 300, int height = 300);

  void setCallback(void(*customCallback)());

  void onChangeInherit(int) override;

private:
  void(*customCallback)() = nullptr;
  cv::Mat hsvImage;
};
